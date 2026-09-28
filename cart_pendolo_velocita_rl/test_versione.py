import math
import unittest
from dataclasses import replace

from stepper import ControlloStepper, ParametriStepper


class ConversioneTest(unittest.TestCase):
    def setUp(self):
        self.p = replace(ParametriStepper.carica(), frequenza_min_hz=0.,
                         frequenza_max_hz=10000., microstepping=16)
        self.c = ControlloStepper(self.p)

    def test_estremi_segno_arresto_e_saturazione(self):
        for u, expected in ((0, 0), (0.5, 5000), (-0.5, -5000), (1, 10000),
                            (-1, -10000), (2, 10000), (-2, -10000)):
            with self.subTest(u=u):
                cmd = self.c.comando(u)
                self.assertEqual(cmd.frequenza_hz, expected)
                self.assertEqual(cmd.frequenza_step_hz, abs(expected))
                self.assertEqual(cmd.direzione, (expected > 0) - (expected < 0))

    def test_minimo_e_zona_morta(self):
        c = ControlloStepper(replace(self.p, frequenza_min_hz=100, zona_morta=0.1))
        for u in (0, 0.05, -0.05, 0.1, -0.1):
            self.assertEqual(c.azione_a_hz(u), 0)
        self.assertAlmostEqual(c.azione_a_hz(0.55), 5050)
        self.assertAlmostEqual(c.azione_a_hz(-0.55), -5050)
        self.assertGreaterEqual(c.azione_a_hz(0.10001), 100)
        self.assertEqual(c.azione_a_hz(1), 10000)

    def test_trasmissione_e_microstepping(self):
        self.assertEqual(self.p.impulsi_per_giro, 3200)
        self.assertAlmostEqual(self.p.metri_per_impulso, 0.00003)
        self.assertAlmostEqual(self.c.comando(1).velocita_carrello_m_s, 0.3)
        half = ControlloStepper(replace(self.p, microstepping=32))
        self.assertAlmostEqual(half.comando(1).velocita_carrello_m_s, 0.15)
        self.assertAlmostEqual(replace(self.p, rapporto_trasmissione=2).metri_per_impulso, 0.000015)

    def test_configurazioni_e_azioni_non_valide(self):
        for change in ({'microstepping': 0}, {'microstepping': 1.5}, {'microstepping': True},
                       {'angolo_passo_gradi': 0.9}, {'passi_per_giro': 0},
                       {'frequenza_max_hz': 0}, {'frequenza_min_hz': -1},
                       {'frequenza_min_hz': 10001}, {'zona_morta': 1},
                       {'rampa_hz_s': 0}, {'rampa_hz_s': -1}, {'rampa_hz_s': float('inf')},
                       {'rapporto_trasmissione': 0}, {'passo_cinghia_m': float('nan')}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                replace(self.p, **change)
        for u in (float('nan'), float('inf'), -float('inf')):
            with self.assertRaises(ValueError):
                self.c.azione_a_hz(u)

    def test_rampa_salita_frenata_inversione_e_reset(self):
        c = ControlloStepper(replace(self.p, rampa_hz_s=1000.))
        self.assertEqual(c.avanza(1, 0.1).frequenza_hz, 100)
        self.assertEqual(c.avanza(1, 0.1).frequenza_hz, 200)
        self.assertEqual(c.avanza(0, 0.1).frequenza_hz, 100)
        self.assertEqual(c.avanza(-1, 0.1).frequenza_hz, 0)
        self.assertEqual(c.avanza(-1, 0.1).frequenza_hz, -100)
        self.assertEqual(c.avanza(-1, 0.1).direzione, -1)
        self.assertEqual(c.reset().frequenza_hz, 0)
        self.assertEqual(c.avanza(1, 0.1).frequenza_hz, 100)

    def test_rampa_senza_overshoot_e_indipendente_dal_passo(self):
        c = ControlloStepper(replace(self.p, rampa_hz_s=1000.))
        for _ in range(10):
            cmd = c.avanza(0.015, 0.02)
        self.assertEqual(cmd.frequenza_hz, 150)
        self.assertAlmostEqual(cmd.velocita_carrello_m_s, 150 * self.p.metri_per_impulso)
        c.reset()
        for _ in range(10):
            cmd = c.avanza(1, 0.002)
        c.reset()
        self.assertEqual(c.avanza(1, 0.02), cmd)

    def test_rampa_attraversa_frequenze_sotto_il_minimo(self):
        c = ControlloStepper(replace(self.p, frequenza_min_hz=3200., rampa_hz_s=1000.))
        self.assertGreaterEqual(c.azione_a_hz(0.01), 3200)
        self.assertEqual(c.avanza(0.01, 0.1).frequenza_hz, 100)
        self.assertEqual(c.avanza(0, 0.2).frequenza_hz, 0)

    def test_dt_non_valido_non_modifica_lo_stato(self):
        self.c.avanza(1, 0.01)
        precedente = self.c.frequenza_hz
        for dt in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                self.c.avanza(1, dt)
            self.assertEqual(self.c.frequenza_hz, precedente)


class AmbienteTest(unittest.TestCase):
    def setUp(self):
        from ambiente import CartPendoloEnv
        self.env = CartPendoloEnv(replace(ParametriStepper.carica(), frequenza_min_hz=0.,
                                         frequenza_max_hz=10000., microstepping=16))
        self.env.reset(seed=42, options={'exact': True})

    def tearDown(self):
        self.env.close()

    def test_rampa_applicata_a_ogni_passo_fisico(self):
        from unittest.mock import patch
        import ambiente
        frequenze = []
        originale = ambiente.mujoco.mj_step

        def misura(model, data):
            frequenze.append(float(data.ctrl[0]) / self.env.controllo.p.metri_per_impulso)
            originale(model, data)

        with patch.object(ambiente.mujoco, 'mj_step', side_effect=misura):
            obs, _, _, _, info = self.env.step([1])
        self.assertEqual(len(frequenze), self.env.frame_skip)
        incremento = self.env.controllo.p.rampa_hz_s * self.env.model.opt.timestep
        for k, hz in enumerate(frequenze, 1):
            self.assertAlmostEqual(hz, k * incremento)
        self.assertEqual(info['requested_frequency_hz'], 10000)
        self.assertAlmostEqual(info['motor_frequency_hz'], self.env.controllo.p.rampa_hz_s * self.env.dt)
        self.assertAlmostEqual(float(obs[-1]), info['motor_frequency_hz'] / 10000)
        _, _, _, _, info = self.env.step([0])
        self.assertEqual(info['motor_frequency_hz'], 0)

    def test_reset_e_finecorsa_interrompono_la_rampa(self):
        self.env.step([1])
        obs, info = self.env.reset(options={'exact': True})
        self.assertEqual(obs[-1], 0)
        self.assertEqual(self.env.controllo.frequenza_hz, 0)
        self.env.step([1])
        self.env.data.qpos[0] = self.env.limite
        _, _, done, _, info = self.env.step([1])
        self.assertTrue(done)
        self.assertEqual(info['motor_frequency_hz'], 0)
        self.assertEqual(self.env.controllo.frequenza_hz, 0)

    def test_gymnasium(self):
        from gymnasium.utils.env_checker import check_env
        check_env(self.env, skip_render_check=True)

    def test_velocita_e_accoppiamento_pendolo(self):
        for sign in (-1, 1):
            self.env.reset(options={'exact': True})
            for _ in range(10):
                obs, reward, term, trunc, info = self.env.step([sign * 0.5])
                self.assertFalse(term or trunc)
            self.assertEqual(info['motor_frequency_hz'], sign * 5000)
            self.assertGreater(sign * info['cart_x'], 0)
            self.assertAlmostEqual(info['cart_velocity_m_s'], sign * 0.15, delta=0.01)
            self.assertGreater(abs(self.env.data.qpos[1] - math.pi), 0.001)
            self.assertEqual(obs.shape, (6,))
        for _ in range(10):
            _, _, _, _, info = self.env.step([0])
        self.assertEqual(info['motor_frequency_hz'], 0)
        self.assertAlmostEqual(info['cart_velocity_m_s'], 0, delta=0.01)

    def test_finecorsa_entrambi_i_versi(self):
        for sign in (-1, 1):
            self.env.reset(options={'exact': True})
            for _ in range(300):
                _, _, term, trunc, info = self.env.step([sign])
                if term or trunc:
                    break
            self.assertTrue(term)
            self.assertEqual(info['termination_reason'], 'finecorsa')
            self.assertTrue(info['right_limit' if sign > 0 else 'left_limit'])
            self.assertEqual(info['motor_frequency_hz'], 0)
            self.assertEqual(info['requested_frequency_hz'], sign * 10000)
            self.assertEqual(float(self.env.data.ctrl[0]), 0)
            with self.assertRaises(RuntimeError):
                self.env.step([sign])

    def test_episodio_completo_e_reset(self):
        import numpy as np
        for k in range(1000):
            obs, reward, term, trunc, info = self.env.step([0.4 * math.sin(k * 0.1)])
            self.assertTrue(np.isfinite(obs).all() and math.isfinite(reward))
            self.assertFalse(term)
            self.assertEqual(trunc, k == 999)
        self.assertEqual(info['motor_frequency_hz'], 0)
        obs, info = self.env.reset(options={'exact': True})
        self.assertEqual(info['requested_frequency_hz'], 0)
        self.assertEqual(info['cart_x'], 0)
        for action in ([], [1, 2], [float('nan')]):
            with self.assertRaises(ValueError):
                self.env.step(action)


if __name__ == '__main__':
    unittest.main()
