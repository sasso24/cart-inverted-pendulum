"""Verifiche riproducibili della meccanica e del contratto per il firmware.

Eseguire prima di riaddestrare dopo modifiche a XML o parametri. I test di
geometria usano MuJoCo senza aprire finestre, quindi funzionano anche senza GUI.
"""
import json
import math
import mujoco
import numpy as np
from stable_baselines3.common.env_checker import check_env
from ambiente import CartPendoloEnv, ROOT
from stepper import Stepper


def main():
    env = CartPendoloEnv()
    check_env(env, warn=True)
    # Reset deterministico: il braccio lungo parte sotto il perno.
    first, _ = env.reset(seed=7)
    second, _ = env.reset(seed=7)
    np.testing.assert_array_equal(first, second)
    exact, _ = env.reset(options={'exact': True})
    assert exact.shape == (6,) and exact.dtype == np.float32
    assert exact[2] == -1. and exact[0] == 0.
    assert env.model.nq == 2 and env.model.nu == 1
    # Il baricentro deve stare dal lato del braccio lungo: zero resta instabile.
    pend = env.model.body('pendolo')
    assert pend.ipos[2] > 0. and math.isclose(pend.mass[0], 0.2, abs_tol=1e-12)

    # Un giro completo in cinque posizioni della guida non deve creare contatti.
    samples = 0
    for x in np.linspace(-env.soglia_finecorsa, env.soglia_finecorsa, 5):
        for angle in np.linspace(-math.pi, math.pi, 361):
            env.data.qpos[:] = [x, angle]
            mujoco.mj_forward(env.model, env.data)
            assert env.data.ncon == 0, (x, angle, env.data.ncon)
            samples += 1

    # Formula passi -> metri, saturazione di velocità, reversibilità del conteggio.
    drive = Stepper()
    assert math.isclose(drive.p.metri_impulso, 0.0000125)
    for _ in range(2000):
        force = drive.avanza(1., 0.002, drive.posizione, drive.velocita)
        assert abs(force) <= drive.p.forza_max
    assert drive.velocita == drive.p.velocita_max
    assert abs(drive.posizione-drive.posizione_continua) <= drive.p.metri_impulso/2 + 1e-12
    drive.reset(0.)
    for _ in range(100): drive.avanza(-1., 0.002, drive.posizione, drive.velocita)
    assert drive.impulsi < 0
    # Accelerazione nulla mantiene la velocità: la frenata deve essere esplicita.
    before = drive.velocita
    drive.avanza(0., 0.002, drive.posizione, drive.velocita)
    assert drive.velocita == before
    drive.avanza(1., 0.002, drive.posizione, drive.velocita)
    assert abs(drive.velocita) < abs(before)

    # Nessuna fuga della posizione fisica verso l'osservazione da impulsi.
    expected, _ = env.reset(options={'exact': True})
    env.data.qpos[0] += 0.01
    np.testing.assert_array_equal(expected, env.observation())

    # Un carrello già sul finecorsa viene fermato al primo sottopasso fisico.
    for sign in (-1, 1):
        env.reset(options={'exact': True})
        env.stepper.reset(sign * (env.soglia_finecorsa + 0.001))
        env.data.qpos[0] = env.stepper.posizione
        mujoco.mj_forward(env.model, env.data)
        _, _, done, _, info = env.step([0.])
        assert done and info['termination_reason'] == 'finecorsa'
        assert env.data.time == env.model.opt.timestep
        assert env.data.ctrl[0] == 0.

    # Un errore eccessivo tra conteggio e meccanica è fuori dal modello nominale.
    env.reset(options={'exact': True})
    env.stepper.reset(0.1)
    assert env.step([0.])[4]['termination_reason'] == 'inseguimento_fuori_modello'
    env.reset(options={'exact': True})
    for bad in ([float('nan')], [0., 1.]):
        try: env.step(bad)
        except ValueError: pass
        else: raise AssertionError('Azione invalida accettata')

    # Caduta libera dal basso: nessun comando, episodio completo senza finecorsa.
    env.reset(options={'exact': True})
    for _ in range(env.max_steps):
        obs, reward, done, truncated, _ = env.step([0.])
        assert np.isfinite(obs).all() and np.isfinite(reward) and not done
    assert truncated
    result = {'status': 'OK', 'collision_free_configurations': samples,
              'observation_size': 6, 'physics_hz': 1/env.model.opt.timestep,
              'control_hz': 1/env.dt, 'metres_per_pulse': drive.p.metri_impulso,
              'pendulum_mass_kg': float(pend.mass[0]), 'com_from_pivot_m': float(pend.ipos[2])}
    folder = ROOT / 'risultati'
    folder.mkdir(exist_ok=True)
    (folder / 'verifica.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    env.close()


if __name__ == '__main__':
    main()
