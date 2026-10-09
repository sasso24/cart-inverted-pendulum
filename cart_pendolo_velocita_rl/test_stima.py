"""Stimatore dello stato e rumore dei sensori richiesto (5-10 mm, 0,5-1 grado)."""
import math
import unittest

import mujoco
import numpy as np

from ambiente import ROOT
from disturbi import ParametriDisturbi, crea_ambiente
from stima import ParametriStimatore, coefficienti_da_modello


class StimaTest(unittest.TestCase):
    def test_coefficienti_toml_coerenti_con_xml(self):
        model = mujoco.MjModel.from_xml_path(str(ROOT / 'cart_pendolo.xml'))
        attesi = coefficienti_da_modello(model)
        p = ParametriStimatore.carica()
        for name, value in attesi.items():
            self.assertAlmostEqual(getattr(p, name), value, delta=1e-3*abs(value),
                                   msg=f'{name}: aggiornare stimatore.toml')

    def test_profilo_rumore_richiesto(self):
        p = ParametriDisturbi.carica(ROOT / 'disturbi.toml')
        self.assertEqual(p.rumore_posizione_m, (0.005, 0.010))
        self.assertEqual(p.rumore_angolo_gradi, (0.5, 1.0))
        env = crea_ambiente(p)
        try:
            for seed in range(10):
                _, info = env.reset(seed=seed)
                d = info['disturbance_parameters']
                self.assertTrue(0.005 <= d['position_noise_m'] <= 0.010)
                self.assertTrue(0.5 <= d['angle_noise_deg'] <= 1.0)
        finally:
            env.close()

    def test_stima_molto_meno_rumorosa_della_derivata(self):
        """Con il rumore massimo, la stima di omega e x batte la derivata grezza."""
        p = ParametriDisturbi(rumore_posizione_m=(0.01, 0.01), rumore_angolo_gradi=(1., 1.))
        env = crea_ambiente(p)
        try:
            env.reset(seed=3)
            err_stima, err_raw, err_x, err_x_raw = [], [], [], []
            precedente = env.misura_grezza
            for k in range(150):
                obs, _, term, trunc, _ = env.step([0.3*math.sin(0.2*k)])
                if term or trunc:
                    break
                x_m, th_m = env.misura_grezza
                omega_raw = (th_m - precedente[1]) / env.dt
                precedente = env.misura_grezza
                if k > 25:  # dopo il transitorio iniziale della stima
                    vera = env.data.qvel[1]
                    err_stima.append(obs[4]*10. - vera)
                    err_raw.append(omega_raw - vera)
                    err_x.append(obs[0]*env.limite - env.data.qpos[0])
                    err_x_raw.append(x_m - env.data.qpos[0])
            rms = lambda e: float(np.sqrt(np.mean(np.square(e))))
            self.assertGreater(len(err_stima), 50)
            self.assertLess(rms(err_stima), 0.3*rms(err_raw))
            self.assertLess(rms(err_x), 0.5*rms(err_x_raw))
        finally:
            env.close()

    def test_parametri_non_validi(self):
        for changes in ({'l_theta': 0.}, {'l_theta': 1.}, {'k_g': float('nan')}, {'l_x': -1}):
            with self.assertRaises(ValueError):
                ParametriStimatore(**changes)
        with self.assertRaises(ValueError):
            ParametriDisturbi(rumore_posizione_m=(0.01, 0.005))


if __name__ == '__main__':
    unittest.main()
