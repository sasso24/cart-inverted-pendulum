"""Gymnasium: swing-up dal basso, cinghia dentata e osservazioni da microstep.

Due soli gradi di libertà fisici (carrello, pendolo). Il generatore STEP aggiunge
uno stato interno: la velocità comandata, esplicitamente osservata dalla policy.
Angolo e velocità angolare simulano un sensore ideale: rumore e ritardi sono ancora
TODO da identificare sul banco. Il conteggio non misura eventuali passi persi.
"""
from pathlib import Path
import math
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
from stepper import Stepper, ParametriStepper

ROOT = Path(__file__).resolve().parent


class CartPendoloEnv(gym.Env):
    metadata = {'render_modes': []}
    frame_skip = 10
    max_steps = 1000 #modificato da 1000 = 20 sec

    def __init__(self, parametri=None):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / 'cart_pendolo.xml'))
        self.data = mujoco.MjData(self.model)
        self.stepper = Stepper(parametri)
        self.model.actuator_ctrlrange[0] = [-self.stepper.p.forza_max, self.stepper.p.forza_max]
        self.limite = float(self.model.joint('scorrimento').range[1])
        self.soglia_finecorsa = self.limite  # Corsa misurata tra gli interventi: 0.924 m.
        self.action_space = spaces.Box(-1., 1., (1,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (6,), dtype=np.float32)
        self.steps = self.stable_steps = self.best_stable_steps = 0
        self.velocita_stimata = 0.0
        self.motivo = ''
        self.max_errore = 0.0

    @property
    def dt(self):
        return self.model.opt.timestep * self.frame_skip

    def observation(self):
        # Ordine e scale fanno parte del contratto della rete, anche su STM32.
        # x e vx derivano esclusivamente dai passi. La verità fisica MuJoCo
        # rimane disponibile per reward, finecorsa e diagnostica di inseguimento.
        angle = self.data.qpos[1]
        return np.array([self.stepper.posizione / self.limite,
                         math.sin(angle), math.cos(angle),
                         self.velocita_stimata / self.stepper.p.velocita_max,
                         self.data.qvel[1] / 10.,
                         self.stepper.velocita / self.stepper.p.velocita_max], dtype=np.float32)

    def info(self):
        angle = math.atan2(math.sin(self.data.qpos[1]), math.cos(self.data.qpos[1]))
        return {'angle_deg': math.degrees(angle), 'cart_x': float(self.data.qpos[0]),
                'estimated_x': self.stepper.posizione, 'step_count': self.stepper.impulsi,
                'command_velocity': self.stepper.velocita,
                'max_tracking_error_m': self.max_errore, 'termination_reason': self.motivo,
                'left_limit': bool(self.data.qpos[0] <= -self.soglia_finecorsa),
                'right_limit': bool(self.data.qpos[0] >= self.soglia_finecorsa),
                'stable_seconds': self.best_stable_steps * self.dt,
                'is_success': bool(self.best_stable_steps * self.dt >= 3.)}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        exact = (options or {}).get('exact', False)
        self.stepper.reset(0. if exact else self.np_random.uniform(-0.1, 0.1))
        self.data.qpos[:] = [self.stepper.posizione,
                            math.pi + (0. if exact else self.np_random.uniform(-0.05, 0.05))]
        self.data.qvel[1] = 0. if exact else self.np_random.uniform(-0.02, 0.02)
        self.steps = self.stable_steps = self.best_stable_steps = 0
        self.velocita_stimata = self.max_errore = 0.0
        self.motivo = ''
        mujoco.mj_forward(self.model, self.data)
        return self.observation(), self.info()

    def step(self, action, *, push=0.):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (1,) or not np.isfinite(action).all() or not math.isfinite(push):
            raise ValueError('Serve un’azione finita di forma (1,) e una spinta finita.')
        u = float(np.clip(action[0], -1., 1.))
        previous_x = self.stepper.posizione
        self.data.qfrc_applied[0] = push
        completed = 0
        # Il servo va aggiornato a OGNI passo fisico. nstep=10 con una sola forza
        # costante rappresenterebbe un attuatore diverso e meno stabile.
        for _ in range(self.frame_skip):
            self.data.ctrl[0] = self.stepper.avanza(u, self.model.opt.timestep,
                                                  self.data.qpos[0], self.data.qvel[0])
            mujoco.mj_step(self.model, self.data)
            completed += 1
            error = abs(self.stepper.posizione - self.data.qpos[0])
            self.max_errore = max(self.max_errore, error)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                raise FloatingPointError('Stato fisico non finito: verificare i parametri.')
            # Finecorsa indipendenti dal conteggio. Interrompiamo già a 500 Hz.
            if abs(self.data.qpos[0]) >= self.soglia_finecorsa:
                self.motivo = 'finecorsa'
            elif error > self.stepper.p.errore_massimo:
                self.motivo = 'inseguimento_fuori_modello'
            if self.motivo:
                self.data.ctrl[0] = 0.
                break
        self.velocita_stimata = (self.stepper.posizione - previous_x) / (completed * self.model.opt.timestep)
        self.steps += 1
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        upright = (math.cos(angle) + 1.) / 2.
        reward = upright * math.exp(-0.25*x*x) * (0.5 + 0.5*math.exp(-0.1*omega*omega)) - 0.005*u*u
        terminated = bool(self.motivo)
        stable = (math.cos(angle) > math.cos(math.radians(12)) and abs(x) < 1.5
                  and abs(omega) < 1. and abs(vx) < 0.75 and not terminated)
        self.stable_steps = self.stable_steps + 1 if stable else 0
        self.best_stable_steps = max(self.best_stable_steps, self.stable_steps)
        return self.observation(), float(reward - 5.*terminated), terminated, self.steps >= self.max_steps, self.info()
