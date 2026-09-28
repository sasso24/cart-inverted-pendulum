"""Verifica configurazione, geometria, finecorsa e comandi del viewer."""
import json
import math
import unittest
import mujoco
import numpy as np
from stable_baselines3.common.env_checker import check_env
from ambiente import CartPendoloEnv, ROOT
from finestra import Comandi, prepare_continuous


def main():
    suite = unittest.defaultTestLoader.discover(str(ROOT), pattern='test_versione.py')
    if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
        raise SystemExit(1)
    env = CartPendoloEnv()
    check_env(env, warn=True)
    samples = 0
    for x in np.linspace(-env.limite + 0.003, env.limite - 0.003, 5):
        for angle in np.linspace(-math.pi, math.pi, 361):
            env.data.qpos[:] = [x, angle]
            mujoco.mj_forward(env.model, env.data)
            assert env.data.ncon == 0, (x, angle, env.data.ncon)
            samples += 1
    pend = env.model.body('pendolo')
    assert pend.ipos[2] > 0 and pend.mass[0] > 0
    for sign in (-1, 1):
        env.reset(options={'exact': True})
        env.data.qpos[0] = sign * env.limite
        _, _, done, _, info = env.step([sign])
        assert done and info['motor_frequency_hz'] == 0
        assert env.data.time == 0
    for sign in (-1, 1):
        prepare_continuous(env)
        commands = Comandi(enabled=False)
        commands.move_cart(env, sign)
        for _ in range(500):
            action = commands.manual_action(env)
            if action is None:
                break
            _, _, done, _, _ = env.step(action)
            assert not done
        assert commands.manual_target is None and 'completato' in commands.last
        assert abs(env.data.qpos[0] - sign * 0.1) < 0.001
    result = {'status': 'OK', 'collision_free_configurations': samples,
              'observation_size': env.observation_space.shape[0],
              'ramp_hz_s': env.controllo.p.rampa_hz_s,
              'physics_hz': 1 / env.model.opt.timestep,
              'control_hz': 1 / env.dt, 'max_step_frequency_hz': env.controllo.p.frequenza_max_hz,
              'metres_per_pulse': env.controllo.p.metri_per_impulso,
              'max_cart_velocity_m_s': env.controllo.p.velocita_max_m_s,
              'pendulum_mass_kg': float(pend.mass[0]), 'manual_moves': 'both directions OK'}
    (ROOT / 'risultati').mkdir(exist_ok=True)
    (ROOT / 'risultati/verifica.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    env.close()


if __name__ == '__main__':
    main()
