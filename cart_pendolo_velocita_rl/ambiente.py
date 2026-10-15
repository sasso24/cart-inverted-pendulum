"""Pendolo MuJoCo con azione di velocita e comando motore in Hz firmati.

Episodio e ricompensa si regolano da config.toml (vedi configurazione.py).
Senza configurazione l'ambiente e quello storico: partenza dall'alto,
ricompensa del 1 ottobre, finecorsa dell'XML.
"""
from pathlib import Path
import math
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
from stepper import ControlloStepper, ParametriStepper

ROOT = Path(__file__).resolve().parent


def ricompensa(x, angle, omega, u, u_precedente):
    """Ricompensa storica (1 ottobre), conservata per riferimento e test.

    Il termine di posizione e stretto (scala 15 cm): con exp(-0.25*x^2) il
    carrello a 40 cm perdeva solo il 4% e nelle prove lunghe derivava fino al finecorsa.
    """
    upright = (math.cos(angle) + 1.) / 2.
    centro = 0.6 + 0.4*math.exp(-(x/0.15)**2)
    fermo = 0.5 + 0.5*math.exp(-0.1*omega*omega)
    return upright*centro*fermo - 0.005*u*u - 0.02*(u - u_precedente)**2


def _fattore(minimo, valore, scala):
    if minimo == 1.:
        return 1.
    return minimo + (1. - minimo)*math.exp(-(valore/scala)**2)


def ricompensa_parametrica(p, x, angle, omega, vx, u, u_precedente, *, k_g=17.1317, limite=0.462):
    """Ricompensa di config.toml [ricompensa]; formula in ParametriRicompensa."""
    angolo = math.remainder(angle, 2*math.pi)
    alto = ((math.cos(angle) + 1.) / 2.)**p.esponente_alto
    r = (alto * _fattore(p.minimo_x, x, p.scala_x_m)
         * _fattore(p.minimo_omega, omega, p.scala_omega_rad_s)
         * _fattore(p.minimo_angolo, math.degrees(angolo), p.scala_angolo_gradi)
         * _fattore(p.minimo_velocita, vx, p.scala_velocita_m_s))
    if p.peso_energia:
        energia = math.cos(angle) + omega*omega/(2.*k_g)
        r += p.peso_energia*math.exp(-((energia - 1.)/p.scala_energia)**2)
    g = ((math.cos(angle) + 1.) / 2.)**p.esponente_penalita_alto if p.esponente_penalita_alto else 1.
    r -= g*(p.peso_azione*u*u + p.peso_variazione*(u - u_precedente)**2)
    if p.peso_inversione and u*u_precedente < 0:
        r -= g*p.peso_inversione
    if p.peso_bordo and abs(x) > p.inizio_bordo_m:
        fascia = max(limite - p.inizio_bordo_m, 1e-6)
        r -= p.peso_bordo*((abs(x) - p.inizio_bordo_m)/fascia)**2
    return r


class CartPendoloEnv(gym.Env):
    metadata = {'render_modes': []}
    frame_skip = 10
    max_steps = 1000

    def __init__(self, parametri=None, *, config_path=None, configurazione=None,
                 addestramento=False):
        super().__init__()
        if parametri is not None and config_path is not None:
            raise ValueError('Specificare parametri oppure config_path, non entrambi.')
        if config_path is not None:
            parametri = ParametriStepper.carica(config_path)
        from configurazione import Configurazione, ParametriEpisodio, ParametriRicompensa
        self.cfg = configurazione
        if parametri is None and configurazione is not None:
            parametri = configurazione.stepper
        # Senza configurazione: episodio e ricompensa storici.
        self.ep = configurazione.episodio if configurazione else ParametriEpisodio()
        self.rp = configurazione.ricompensa if configurazione else None
        self.k_g = configurazione.stimatore.k_g if configurazione else 17.1317
        # In addestramento (solo swing-up) una parte degli episodi parte in alto.
        self.addestramento = addestramento
        self.controllo = ControlloStepper(parametri)
        xml = configurazione.modello.file_xml if configurazione else 'cart_pendolo.xml'
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / xml))
        self.data = mujoco.MjData(self.model)
        # limite: scala della posizione nella rete (finecorsa XML), invariata.
        # limite_corsa: arresto dell'episodio, uguale o piu interno.
        self.limite = float(self.model.joint('scorrimento').range[1])
        self.limite_corsa = self.ep.limite_corsa_m or self.limite
        if self.limite_corsa > self.limite:
            raise ValueError('limite_corsa_m oltre il finecorsa dell XML.')
        if self.ep.durata_s != 20.0:
            self.max_steps = int(round(self.ep.durata_s / self.dt))
        self.action_space = spaces.Box(-1., 1., (1,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (6,), dtype=np.float32)
        self.steps = self.stable_steps = self.best_stable_steps = 0
        self.u_precedente = 0.
        self.motivo = ''
        self.comando_motore = self.controllo.comando(0.)
        self.frequenza_richiesta_hz = 0.
        self.angolo_iniziale = 0.
        self._episode_done = True

    @property
    def dt(self):
        return self.model.opt.timestep * self.frame_skip

    def observation(self):
        # Sensori ideali dello stato fisico; nessun conteggio STEP fittizio.
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        return np.array([x / self.limite, math.sin(angle), math.cos(angle),
                         vx / self.controllo.p.velocita_max_m_s, omega / self.ep.scala_omega_rad_s,
                         self.comando_motore.frequenza_hz / self.controllo.p.frequenza_max_hz],
                        dtype=np.float32)

    def info(self):
        angle = math.atan2(math.sin(self.data.qpos[1]), math.cos(self.data.qpos[1]))
        c = self.comando_motore
        return {'motor_frequency_hz': c.frequenza_hz,
                'step_frequency_hz': c.frequenza_step_hz,
                'direction': c.direzione,
                'requested_frequency_hz': self.frequenza_richiesta_hz,
                'command_velocity_m_s': c.velocita_carrello_m_s,
                'cart_velocity_m_s': float(self.data.qvel[0]),
                'cart_x': float(self.data.qpos[0]), 'angle_deg': math.degrees(angle),
                'termination_reason': self.motivo,
                'left_limit': bool(self.data.qpos[0] <= -self.limite_corsa),
                'right_limit': bool(self.data.qpos[0] >= self.limite_corsa),
                'stable_seconds': self.best_stable_steps * self.dt,
                'final_stable_seconds': self.stable_steps * self.dt,
                'is_success': bool(self.best_stable_steps * self.dt >= 3.)}

    def successo(self, terminated):
        """Equilibrio ininterrotto da tempo_max_salita_s (0 = sempre) fino alla fine."""
        richiesti = self.steps - int(round(self.ep.tempo_max_salita_s / self.dt))
        return bool(not terminated and self.steps >= self.max_steps
                    and self.stable_steps >= max(richiesti, 1))

    def _stop(self):
        self.comando_motore = self.controllo.reset()
        self.data.ctrl[0] = 0.

    def _stato_iniziale(self, exact):
        ep, rng = self.ep, self.np_random
        centro = 0.
        if ep.swing_up:
            centro = math.radians(ep.angolo_iniziale_gradi)
            if self.addestramento and ep.frazione_partenza_alto > 0 \
                    and rng.random() < ep.frazione_partenza_alto:
                centro = 0.
        x = 0. if exact else rng.uniform(-ep.dispersione_x_m, ep.dispersione_x_m)
        angolo = centro + (0. if exact else rng.uniform(-ep.dispersione_angolo_rad,
                                                         ep.dispersione_angolo_rad))
        omega = 0. if exact else rng.uniform(-ep.dispersione_omega_rad_s, ep.dispersione_omega_rad_s)
        return x, angolo, omega

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        exact = (options or {}).get('exact', False)
        x, angolo, omega = self._stato_iniziale(exact)
        self.data.qpos[:] = [x, angolo]
        self.data.qvel[1] = omega
        self.angolo_iniziale = angolo
        self.steps = self.stable_steps = self.best_stable_steps = 0
        self.u_precedente = 0.
        self.motivo = ''
        self.frequenza_richiesta_hz = 0.
        self._stop()
        self._episode_done = False
        mujoco.mj_forward(self.model, self.data)
        return self.observation(), self.info()

    def _fuori_corsa(self):
        return abs(self.data.qpos[0]) >= self.limite_corsa

    def _ricompensa(self, u):
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        if self.rp is None:
            return ricompensa(x, angle, omega, u, self.u_precedente)
        return ricompensa_parametrica(self.rp, x, angle, omega, vx, u, self.u_precedente,
                                      k_g=self.k_g, limite=self.limite_corsa)

    def _fine_passo(self, u):
        """Parte comune a nominale e disturbato dopo l'integrazione."""
        self.steps += 1
        if (not self.motivo and self.ep.giri_max
                and abs(self.data.qpos[1] - self.angolo_iniziale) > 2*math.pi*self.ep.giri_max):
            self.motivo = 'troppi giri'
        terminated = bool(self.motivo)
        truncated = self.steps >= self.max_steps
        self._episode_done = terminated or truncated
        if self._episode_done:
            self._stop()
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        reward = self._ricompensa(u)
        self.u_precedente = u
        ep = self.ep
        stable = (math.cos(angle) > math.cos(math.radians(ep.stabile_angolo_gradi))
                  and abs(x) < self.limite_corsa and abs(omega) < ep.stabile_omega_rad_s
                  and abs(vx) < ep.stabile_velocita_m_s and not terminated)
        self.stable_steps = self.stable_steps + 1 if stable else 0
        self.best_stable_steps = max(self.best_stable_steps, self.stable_steps)
        return float(reward - ep.penalita_finecorsa*terminated), terminated, truncated

    def step(self, action, *, push=0.):
        if self._episode_done:
            raise RuntimeError('Chiamare reset() prima di iniziare un nuovo episodio.')
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (1,) or not np.isfinite(action).all() or not math.isfinite(push):
            raise ValueError('Serve un’azione finita di forma (1,) e una spinta finita.')
        u = float(np.clip(action[0], -1., 1.))
        self.frequenza_richiesta_hz = self.controllo.azione_a_hz(u)
        self.data.qfrc_applied[0] = push
        for _ in range(self.frame_skip):
            if self._fuori_corsa():
                self.motivo = 'finecorsa'
                break
            self.comando_motore = self.controllo.avanza(u, self.model.opt.timestep)
            self.data.ctrl[0] = self.comando_motore.velocita_carrello_m_s
            mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                self._stop()
                self._episode_done = True
                raise FloatingPointError('Stato fisico non finito.')
            if self._fuori_corsa():
                self.motivo = 'finecorsa'
                break
        reward, terminated, truncated = self._fine_passo(u)
        return self.observation(), reward, terminated, truncated, self.info()
