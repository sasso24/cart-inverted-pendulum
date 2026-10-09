"""Stress riproducibile sul modello nominale, senza cambiarne XML o firmware.

I modi sono oscillatori equivalenti di forza/coppia, non una simulazione FEM
dell'asta o un modello elettromeccanico dello stepper.
"""
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import tomllib

import mujoco
import numpy as np

from ambiente import CartPendoloEnv, ricompensa
from stima import StimatoreStato


@dataclass(frozen=True)
class ParametriDisturbi:
    intensita: float = 0.35
    massa_rel: float = 0.10
    smorzamento_rel: float = 0.25
    guadagno_motore_rel: float = 0.15
    ritardo_max_s: float = 0.006
    frequenze_min_hz: tuple = (8., 20., 3.)
    frequenze_max_hz: tuple = (18., 40., 10.)
    zeta_min: float = 0.08
    zeta_max: float = 0.25
    forza_cinghia_n: float = 0.20
    forza_motore_n: float = 0.10
    coppia_asta_nm: float = 0.003
    accelerazione_scala_m_s2: float = 10.
    eccitazione_casuale: float = 0.25
    correlazione_s: float = 0.04
    # Deviazione standard del rumore, estratta per episodio in [min, max].
    # NON scalata da intensita: sono i valori richiesti per i sensori.
    rumore_posizione_m: tuple = (0.005, 0.010)
    rumore_angolo_gradi: tuple = (0.5, 1.0)
    offset_angolo_rad: float = math.radians(0.5)
    encoder_conteggi_giro: int = 2400

    def __post_init__(self):
        for name, value in vars(self).items():
            if name.startswith('frequenze_') or name.startswith('rumore_'):
                n = 3 if name.startswith('frequenze_') else 2
                if not isinstance(value, (list, tuple)) or len(value) != n:
                    raise ValueError(f'{name}: servono {n} valori.')
                object.__setattr__(self, name, tuple(value))
                values = value
            else:
                values = (value,)
            if any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) or v < 0 for v in values):
                raise ValueError(f'{name}: servono numeri finiti non negativi.')
        if not 0 <= self.intensita <= 1:
            raise ValueError('intensita deve essere in [0, 1].')
        if any(getattr(self, n) >= 1 for n in
               ('massa_rel', 'smorzamento_rel', 'guadagno_motore_rel')):
            raise ValueError('Le variazioni relative devono essere inferiori a 1.')
        if not 0 < self.zeta_min <= self.zeta_max < 1:
            raise ValueError('Serve 0 < zeta_min <= zeta_max < 1.')
        if any(not 0 < lo <= hi for lo, hi in
               zip(self.frequenze_min_hz, self.frequenze_max_hz)):
            raise ValueError('Intervalli di frequenza non validi.')
        if any(lo > hi for lo, hi in (self.rumore_posizione_m, self.rumore_angolo_gradi)):
            raise ValueError('Rumore: serve [minimo, massimo] con minimo <= massimo.')
        if self.correlazione_s <= 0 or self.accelerazione_scala_m_s2 <= 0:
            raise ValueError('Correlazione e scala accelerazione devono essere positive.')
        if type(self.encoder_conteggi_giro) is not int or self.encoder_conteggi_giro < 1:
            raise ValueError('encoder_conteggi_giro deve essere un intero positivo.')
        if self.ritardo_max_s > 0.1:
            raise ValueError('ritardo_max_s deve essere <= 0.1 s.')

    @classmethod
    def carica(cls, path):
        with Path(path).open('rb') as stream:
            config = tomllib.load(stream)
        if set(config) != {'disturbi'}:
            raise ValueError('Serve solo la sezione [disturbi].')
        return cls(**config['disturbi'])


class ModiSmorzati:
    """y'' + 2*zeta*w*y' + w²*y = w²*ingresso, soluzione esatta per ZOH.

    y e adimensionale; la forza/coppia viene scalata e limitata separatamente.
    La discretizzazione esatta evita instabilita numerica dei modi veloci.
    """
    def __init__(self, frequenze, zeta, dt):
        w = 2 * np.pi * np.asarray(frequenze)
        a = np.asarray(zeta) * w
        wd = np.sqrt(w*w - a*a)
        decay, c, s = np.exp(-a*dt), np.cos(wd*dt), np.sin(wd*dt)/wd
        self.e00, self.e01 = decay*(c+a*s), decay*s
        self.e10, self.e11 = -decay*w*w*s, decay*(c-a*s)
        self.y = np.zeros_like(w)
        self.v = np.zeros_like(w)

    def step(self, ingresso):
        delta = self.y - ingresso
        y = ingresso + self.e00*delta + self.e01*self.v
        self.v = self.e10*delta + self.e11*self.v
        self.y = y
        return y


class CartPendoloDisturbatoEnv(CartPendoloEnv):
    def __init__(self, disturbi, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.disturbi = disturbi
        if max(disturbi.frequenze_max_hz) * self.model.opt.timestep > 0.1:
            raise ValueError('Servono almeno 10 passi fisici per periodo dei modi.')
        self._nominale = {n: getattr(self.model, n).copy() for n in
                          ('body_mass', 'body_inertia', 'dof_damping',
                           'actuator_gainprm', 'actuator_biasprm')}
        self._obs = None
        self.stimatore = StimatoreStato()
        self.parametri_episodio = {}
        self.forza_disturbo_n = self.coppia_disturbo_nm = 0.

    def observation(self):
        return super().observation() if self._obs is None else self._obs.copy()

    def info(self):
        return {**super().info(), 'disturbance_force_n': self.forza_disturbo_n,
                'disturbance_torque_nm': self.coppia_disturbo_nm,
                'disturbance_parameters': self.parametri_episodio.copy()}

    def _stop(self):
        super()._stop()
        self.data.qfrc_applied[:] = 0.
        if hasattr(self, '_azioni'):
            self._azioni.clear()

    def reset(self, *, seed=None, options=None):
        # Il reset nominale estrae prima lo stato iniziale: a parita di seed
        # il confronto nominale/disturbato parte dalle stesse qpos e qvel.
        for name, values in self._nominale.items():
            getattr(self.model, name)[:] = values
        mujoco.mj_setConst(self.model, self.data)
        self._obs = None
        super().reset(seed=seed, options=options)
        p, rng, s = self.disturbi, self.np_random, self.disturbi.intensita
        scales = rng.uniform(1-s*p.massa_rel, 1+s*p.massa_rel, 2)
        for name, scale in zip(('carrello', 'pendolo'), scales):
            bid = self.model.body(name).id
            self.model.body_mass[bid] *= scale
            self.model.body_inertia[bid] *= scale
        damping = rng.uniform(1-s*p.smorzamento_rel, 1+s*p.smorzamento_rel, 2)
        self.model.dof_damping[:] *= damping
        gain = rng.uniform(1-s*p.guadagno_motore_rel, 1+s*p.guadagno_motore_rel)
        self.model.actuator_gainprm[0, 0] *= gain
        self.model.actuator_biasprm[0, 2] *= gain
        # mj_setConst puo usare qpos0: preservare lo stato estratto dal reset.
        qpos, qvel = self.data.qpos.copy(), self.data.qvel.copy()
        mujoco.mj_setConst(self.model, self.data)
        self.data.qpos[:], self.data.qvel[:] = qpos, qvel
        mujoco.mj_forward(self.model, self.data)
        freq = rng.uniform(p.frequenze_min_hz, p.frequenze_max_hz)
        zeta = rng.uniform(p.zeta_min, p.zeta_max, 3)
        self.modi = ModiSmorzati(freq, zeta, self.model.opt.timestep)
        self._ampiezze = s*rng.uniform(0.5, 1., 3)*np.array(
            [p.forza_cinghia_n, p.forza_motore_n, p.coppia_asta_nm])
        delay_max = int(math.floor(s*p.ritardo_max_s / self.model.opt.timestep + 1e-9))
        delay = int(rng.integers(delay_max+1))
        self._azioni = deque([0.]*delay)
        self._offset = rng.uniform(-s*p.offset_angolo_rad, s*p.offset_angolo_rad)
        self._sigma_x = rng.uniform(*p.rumore_posizione_m)
        self._sigma_theta = math.radians(rng.uniform(*p.rumore_angolo_gradi))
        self._colore = np.zeros(3)
        self._rho = math.exp(-self.model.opt.timestep / p.correlazione_s)
        self._ultima_velocita = 0.
        self.forza_disturbo_n = self.coppia_disturbo_nm = 0.
        self.parametri_episodio = {
            'intensita': s, 'mass_scales': scales.tolist(), 'damping_scales': damping.tolist(),
            'motor_gain_scale': gain, 'frequencies_hz': freq.tolist(), 'zeta': zeta.tolist(),
            'amplitude_limits': self._ampiezze.tolist(),
            'delay_s': delay*self.model.opt.timestep, 'angle_offset_rad': self._offset,
            'position_noise_m': self._sigma_x, 'angle_noise_deg': math.degrees(self._sigma_theta)}
        self._misura(iniziale=True)
        return self.observation(), self.info()

    def _misura(self, iniziale=False):
        """Lettura rumorosa a 50 Hz, poi correzione dello stimatore (stima.py).

        La rete non vede le misure grezze: derivarle amplificherebbe il rumore.
        """
        p = self.disturbi
        x, theta = self.data.qpos
        x += self.np_random.normal(0., self._sigma_x)
        theta += self._offset + self.np_random.normal(0., self._sigma_theta)
        tick = 2*math.pi/p.encoder_conteggi_giro
        theta = round(theta/tick)*tick
        self.misura_grezza = (x, theta)
        if iniziale:
            self.stimatore.reset(x, theta, self.comando_motore.velocita_carrello_m_s)
        else:
            self.stimatore.correggi(x, theta)
        st = self.stimatore
        self._obs = np.array([st.x/self.limite, math.sin(st.theta), math.cos(st.theta),
                              st.velocita/self.controllo.p.velocita_max_m_s, st.omega/10.,
                              self.comando_motore.frequenza_hz/self.controllo.p.frequenza_max_hz],
                             dtype=np.float32)

    def _forze(self, velocita, push):
        p = self.disturbi
        accel = (velocita-self._ultima_velocita)/self.model.opt.timestep
        self._ultima_velocita = velocita
        self._colore = (self._rho*self._colore + math.sqrt(1-self._rho**2)
                        * self.np_random.normal(size=3))
        drive = np.clip(accel/p.accelerazione_scala_m_s2, -1., 1.)
        response = self.modi.step(drive + p.eccitazione_casuale*self._colore)
        loads = self._ampiezze*np.tanh(response)
        self.forza_disturbo_n = float(loads[0]+loads[1])
        self.coppia_disturbo_nm = float(loads[2])
        self.data.qfrc_applied[:] = [push+self.forza_disturbo_n, self.coppia_disturbo_nm]

    def step(self, action, *, push=0.):
        # Stesso ciclo/ricompensa del nominale, con ritardo e forze a 500 Hz.
        # Il nominale resta in ambiente.py per conservare i checkpoint esistenti.
        if self._episode_done:
            raise RuntimeError('Chiamare reset() prima di iniziare un nuovo episodio.')
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (1,) or not np.isfinite(action).all() or not math.isfinite(push):
            raise ValueError('Serve un’azione finita di forma (1,) e una spinta finita.')
        u = float(np.clip(action[0], -1., 1.))
        self.frequenza_richiesta_hz = self.controllo.azione_a_hz(u)
        start = self.data.time
        for _ in range(self.frame_skip):
            if abs(self.data.qpos[0]) >= self.limite:
                self.motivo = 'finecorsa'
                break
            self._azioni.append(u)
            delayed = self._azioni.popleft()
            self.comando_motore = self.controllo.avanza(delayed, self.model.opt.timestep)
            self.data.ctrl[0] = self.comando_motore.velocita_carrello_m_s
            # Sul firmware: stessa previsione nell'interrupt della rampa a 500 Hz.
            self.stimatore.predici(self.comando_motore.velocita_carrello_m_s, self.model.opt.timestep)
            self._forze(self.data.ctrl[0], push)
            mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                self._stop()
                self._episode_done = True
                raise FloatingPointError('Stato fisico non finito.')
            if abs(self.data.qpos[0]) >= self.limite:
                self.motivo = 'finecorsa'
                break
        self.steps += 1
        terminated, truncated = bool(self.motivo), self.steps >= self.max_steps
        self._episode_done = terminated or truncated
        if self._episode_done:
            self._stop()
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        reward = ricompensa(x, angle, omega, u, self.u_precedente)
        self.u_precedente = u
        stable = (math.cos(angle) > math.cos(math.radians(12)) and abs(x) < self.limite
                  and abs(omega) < 1. and abs(vx) < 0.75 and not terminated)
        self.stable_steps = self.stable_steps+1 if stable else 0
        self.best_stable_steps = max(self.best_stable_steps, self.stable_steps)
        if self.data.time > start:
            self._misura()
        else:
            self._misura(iniziale=True)
        return self.observation(), float(reward-5.*terminated), terminated, truncated, self.info()


def crea_ambiente(disturbi=None, **kwargs):
    if disturbi is None or disturbi.intensita == 0:
        return CartPendoloEnv(**kwargs)
    return CartPendoloDisturbatoEnv(disturbi, **kwargs)
