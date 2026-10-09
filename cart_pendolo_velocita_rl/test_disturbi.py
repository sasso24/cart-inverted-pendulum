from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from ambiente import CartPendoloEnv, ROOT
from disturbi import ParametriDisturbi, ModiSmorzati, crea_ambiente


class DisturbiTest(unittest.TestCase):
    def test_configurazione_e_limiti(self):
        p = ParametriDisturbi.carica(ROOT / 'disturbi.toml')
        self.assertEqual(p, ParametriDisturbi(**json.loads(json.dumps(asdict(p)))))
        for changes in ({'intensita': -1}, {'intensita': 1.1}, {'massa_rel': 1},
                        {'zeta_min': 0}, {'zeta_max': 1}, {'ritardo_max_s': 0.2},
                        {'frequenze_min_hz': [1, 2]}, {'frequenze_max_hz': [1, 1, 1]},
                        {'encoder_conteggi_giro': 2.5}, {'correlazione_s': 0},
                        {'forza_cinghia_n': float('nan')}, {'intensita': True}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(p, **changes)
        with self.assertRaises(ValueError):
            crea_ambiente(replace(p, frequenze_max_hz=(100., 100., 100.)))

    def test_intensita_zero_identica_al_nominale(self):
        a = CartPendoloEnv()
        b = crea_ambiente(replace(ParametriDisturbi(), intensita=0.))
        try:
            np.testing.assert_array_equal(a.reset(seed=7)[0], b.reset(seed=7)[0])
            for k in range(200):
                action = [0.1*math.sin(k*0.3)]
                oa, ra, ta, xa, ia = a.step(action, push=0.01)
                ob, rb, tb, xb, ib = b.step(action, push=0.01)
                np.testing.assert_array_equal(oa, ob)
                self.assertEqual((ra, ta, xa, ia), (rb, tb, xb, ib))
                if ta or xa:
                    break
        finally:
            a.close()
            b.close()

    def test_modi_oscillano_si_smorzano_e_non_dipendono_dal_passo(self):
        coarse = ModiSmorzati([10.], [0.1], 0.002)
        fine = ModiSmorzati([10.], [0.1], 0.001)
        for _ in range(20):
            coarse.step(1.)
            fine.step(1.)
            fine.step(1.)
        np.testing.assert_allclose(coarse.y, fine.y, atol=1e-12)
        response = [float(coarse.step(0.)[0]) for _ in range(1500)]
        self.assertLess(min(response), -0.1)
        self.assertGreater(max(response), 0.1)
        self.assertLess(abs(response[-1]), 1e-7)

    def test_seed_reset_parametri_e_letture_ripetute(self):
        env = crea_ambiente(ParametriDisturbi())
        try:
            first, initial = env.reset(seed=15)
            masses = env.model.body_mass.copy()
            state = env.np_random.bit_generator.state
            for _ in range(4):
                np.testing.assert_array_equal(env.observation(), first)
                env.info()
            self.assertEqual(state, env.np_random.bit_generator.state)
            rollout = [env.step([0.1*math.sin(k)]) for k in range(80)]
            again, info = env.reset(seed=15)
            np.testing.assert_array_equal(first, again)
            np.testing.assert_array_equal(masses, env.model.body_mass)
            self.assertEqual(initial, info)
            for k, expected in enumerate(rollout):
                actual = env.step([0.1*math.sin(k)])
                np.testing.assert_array_equal(actual[0], expected[0])
                self.assertEqual(actual[1:], expected[1:])
            for seed in range(20):
                _, info = env.reset(seed=seed)
                for i, name in enumerate(('carrello', 'pendolo')):
                    bid = env.model.body(name).id
                    ratio = env.model.body_mass[bid]/env._nominale['body_mass'][bid]
                    self.assertAlmostEqual(ratio, info['disturbance_parameters']['mass_scales'][i])
                    self.assertLessEqual(abs(ratio-1), 0.035+1e-12)
                    np.testing.assert_allclose(env.model.body_inertia[bid],
                                               env._nominale['body_inertia'][bid]*ratio)
        finally:
            env.close()

    def test_forze_500_hz_rumore_non_modifica_la_fisica(self):
        import disturbi
        p = replace(ParametriDisturbi(), massa_rel=0., smorzamento_rel=0.,
                    guadagno_motore_rel=0., ritardo_max_s=0., forza_cinghia_n=0.,
                    forza_motore_n=0., coppia_asta_nm=0.)
        env, nominal = crea_ambiente(p), CartPendoloEnv()
        try:
            env.reset(seed=9)
            nominal.reset(seed=9)
            count = []
            original = disturbi.mujoco.mj_step
            def record(model, data):
                count.append(data.qfrc_applied.copy())
                original(model, data)
            with patch.object(disturbi.mujoco, 'mj_step', side_effect=record):
                obs, reward, _, _, _ = env.step([0.1], push=0.05)
            self.assertEqual(len(count), env.frame_skip)
            np.testing.assert_allclose(count, np.tile([0.05, 0.], (env.frame_skip, 1)))
            ideal, expected_reward, _, _, _ = nominal.step([0.1], push=0.05)
            np.testing.assert_allclose(env.data.qpos, nominal.data.qpos, atol=1e-14)
            np.testing.assert_allclose(env.data.qvel, nominal.data.qvel, atol=1e-14)
            self.assertAlmostEqual(reward, expected_reward)
            self.assertFalse(np.array_equal(obs, ideal))
            self.assertAlmostEqual(float(obs[1]**2+obs[2]**2), 1., places=6)
        finally:
            env.close()
            nominal.close()

    def test_ritardo_limiti_arresto_e_gymnasium(self):
        from gymnasium.utils.env_checker import check_env
        env = crea_ambiente(replace(ParametriDisturbi(), intensita=1., ritardo_max_s=0.02))
        try:
            check_env(env, skip_render_check=True)
            for seed in range(10):
                _, info = env.reset(seed=seed)
                delay = round(info['disturbance_parameters']['delay_s']/env.model.opt.timestep)
                if delay:
                    break
            self.assertGreater(delay, 0)
            import disturbi
            controls = []
            original = disturbi.mujoco.mj_step
            def record(model, data):
                controls.append(data.ctrl[0])
                original(model, data)
            with patch.object(disturbi.mujoco, 'mj_step', side_effect=record):
                env.step([0.3])
                env.step([0.3])
            np.testing.assert_array_equal(controls[:delay], np.zeros(delay))
            self.assertGreater(controls[delay], 0.)
            for sign in (-1, 1):
                env.reset(seed=0)
                for k in range(60):
                    env.step([0.1*math.sin(k)])
                    self.assertLessEqual(abs(env.forza_disturbo_n), 0.30)
                    self.assertLessEqual(abs(env.coppia_disturbo_nm), 0.003)
                env.data.qpos[0] = sign*env.limite
                obs, _, done, _, _ = env.step([1.])
                self.assertTrue(done)
                self.assertTrue(np.isfinite(obs).all())
                self.assertEqual(obs[-1], 0.)
                self.assertEqual(env.data.ctrl[0], 0.)
                np.testing.assert_array_equal(env.data.qfrc_applied, [0., 0.])
                self.assertEqual(len(env._azioni), 0)
                with self.assertRaises(RuntimeError):
                    env.step([0.])
                env.reset(seed=0)
                self.assertEqual(env.forza_disturbo_n, 0.)
                np.testing.assert_array_equal(env.modi.y, np.zeros(3))
        finally:
            env.close()

    def test_contratti_nominali_e_disturbati(self):
        from addestra import check_contract, contract, profilo_salvato
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            (path/'config.json').write_text(json.dumps({'environment': contract()}))
            check_contract(path)
            self.assertIsNone(profilo_salvato(path))
            p = ParametriDisturbi()
            config = {'environment': contract(p)}
            (path/'config.json').write_text(json.dumps(config))
            check_contract(path)
            self.assertEqual(profilo_salvato(path), p)
            config['environment']['sha256']['disturbi.py'] = 'diverso'
            (path/'config.json').write_text(json.dumps(config))
            with self.assertRaises(ValueError):
                check_contract(path)


if __name__ == '__main__':
    unittest.main()
