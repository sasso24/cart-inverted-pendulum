"""Pendolo MuJoCo con azione di velocita e comando motore in Hz firmati."""
from pathlib import Path
import math
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
from stepper import ControlloStepper, ParametriStepper

ROOT = Path(__file__).resolve().parent


class CartPendoloEnv(gym.Env):
    metadata = {'render_modes': []}
    frame_skip = 10
    max_steps = 1000

    def __init__(self, parametri=None, *, config_path=None):
        super().__init__()
        if parametri is not None and config_path is not None:
            raise ValueError('Specificare parametri oppure config_path, non entrambi.')
        if config_path is not None:
            parametri = ParametriStepper.carica(config_path)
        self.controllo = ControlloStepper(parametri)
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / 'cart_pendolo.xml'))
        self.data = mujoco.MjData(self.model)
        self.limite = float(self.model.joint('scorrimento').range[1])
        self.action_space = spaces.Box(-1., 1., (1,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (6,), dtype=np.float32)
        self.steps = self.stable_steps = self.best_stable_steps = 0
        self.motivo = ''
        self.comando_motore = self.controllo.comando(0.)
        self.frequenza_richiesta_hz = 0.
        self._episode_done = True

    @property
    def dt(self):
        return self.model.opt.timestep * self.frame_skip

    def observation(self):
        # Sensori ideali dello stato fisico; nessun conteggio STEP fittizio.
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        return np.array([x / self.limite, math.sin(angle), math.cos(angle),
                         vx / self.controllo.p.velocita_max_m_s, omega / 10.,
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
                'left_limit': bool(self.data.qpos[0] <= -self.limite),
                'right_limit': bool(self.data.qpos[0] >= self.limite),
                'stable_seconds': self.best_stable_steps * self.dt,
                'is_success': bool(self.best_stable_steps * self.dt >= 3.)}

    def _stop(self):
        self.comando_motore = self.controllo.reset()
        self.data.ctrl[0] = 0.

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        exact = (options or {}).get('exact', False)
        self.data.qpos[:] = [0. if exact else self.np_random.uniform(-0.1, 0.1),
                            math.pi + (0. if exact else self.np_random.uniform(-0.05, 0.05))]
        self.data.qvel[1] = 0. if exact else self.np_random.uniform(-0.02, 0.02)
        self.steps = self.stable_steps = self.best_stable_steps = 0
        self.motivo = ''
        self.frequenza_richiesta_hz = 0.
        self._stop()
        self._episode_done = False
        mujoco.mj_forward(self.model, self.data)
        return self.observation(), self.info()

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
            if abs(self.data.qpos[0]) >= self.limite:
                self.motivo = 'finecorsa'
                break
            self.comando_motore = self.controllo.avanza(u, self.model.opt.timestep)
            self.data.ctrl[0] = self.comando_motore.velocita_carrello_m_s
            mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                self._stop()
                self._episode_done = True
                raise FloatingPointError('Stato fisico non finito.')
            if abs(self.data.qpos[0]) >= self.limite:
                self.motivo = 'finecorsa'
                break
        self.steps += 1
        terminated = bool(self.motivo)
        truncated = self.steps >= self.max_steps
        self._episode_done = terminated or truncated
        if self._episode_done:
            self._stop()
        x, angle = self.data.qpos
        vx, omega = self.data.qvel
        upright = (math.cos(angle) + 1.) / 2.
        reward = upright * math.exp(-0.25*x*x) * (0.5 + 0.5*math.exp(-0.1*omega*omega)) - 0.005*u*u
        stable = (math.cos(angle) > math.cos(math.radians(12)) and abs(x) < self.limite
                  and abs(omega) < 1. and abs(vx) < 0.75 and not terminated)
        self.stable_steps = self.stable_steps + 1 if stable else 0
        self.best_stable_steps = max(self.best_stable_steps, self.stable_steps)
        return self.observation(), float(reward - 5.*terminated), terminated, truncated, self.info()
