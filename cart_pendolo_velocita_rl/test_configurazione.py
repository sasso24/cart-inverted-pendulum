"""Configurazione unica (config.toml), swing-up, ricompensa parametrica, guasti sensori."""
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from ambiente import CartPendoloEnv, ROOT, ricompensa, ricompensa_parametrica
from configurazione import (Configurazione, ParametriRicompensa, ParametriSensori,
                            configurazione_ambiente, configurazione_modello)
from disturbi import ParametriDisturbi, crea_ambiente


class ConfigurazioneTest(unittest.TestCase):
    def setUp(self):
        self.cfg = Configurazione.carica(ROOT / 'config.toml')

    def test_file_unico_completo(self):
        c = self.cfg
        self.assertTrue(c.episodio.swing_up)
        self.assertEqual(c.episodio.angolo_iniziale_gradi, -180.)
        self.assertIsNotNone(c.disturbi)
        self.assertTrue(c.sensori.attivi)
        self.assertEqual(c.addestramento.strati, (128, 128))
        # Andata e ritorno JSON (config.json dei modelli) senza perdite.
        self.assertEqual(Configurazione.da_dict(c.a_dict()), c)

    def test_chiavi_sconosciute_rifiutate(self):
        d = self.cfg.a_dict()
        d['ricompensa']['peso_sbagliato'] = 1.
        with self.assertRaises(ValueError):
            Configurazione.da_dict(d)
        d = self.cfg.a_dict()
        d['sezione_inventata'] = {}
        with self.assertRaises(ValueError):
            Configurazione.da_dict(d)

    def test_disturbi_disattivabili(self):
        d = self.cfg.a_dict()
        d['disturbi']['attivi'] = False
        self.assertIsNone(Configurazione.da_dict(d).disturbi)

    def test_ricompensa_default_uguale_alla_storica(self):
        rng = np.random.default_rng(0)
        p = ParametriRicompensa()
        for _ in range(2000):
            x, a, w, v, u, up = rng.uniform(-1, 1, 6) * [0.4, 4, 8, 0.5, 1, 1]
            self.assertAlmostEqual(ricompensa(x, a, w, u, up),
                                   ricompensa_parametrica(p, x, a, w, v, u, up), places=12)

    def test_ricompensa_privilegia_omega(self):
        p = self.cfg.ricompensa
        fermo_inclinato = ricompensa_parametrica(p, 0., math.radians(5), 0., 0., 0., 0.)
        dritto_in_moto = ricompensa_parametrica(p, 0., 0., 1.5, 0., 0., 0.)
        # 5 gradi di errore costano poco, 1,5 rad/s costano molto.
        self.assertGreater(fermo_inclinato, 0.99)
        self.assertLess(dritto_in_moto, 0.75)
        # Penalita su inversioni e bordo.
        self.assertLess(ricompensa_parametrica(p, 0., 0., 0., 0., 0.1, -0.1),
                        ricompensa_parametrica(p, 0., 0., 0., 0., 0.1, 0.1))
        self.assertLess(ricompensa_parametrica(p, 0.38, 0., 0., 0., 0., 0., limite=0.4),
                        ricompensa_parametrica(p, 0.25, 0., 0., 0., 0., 0., limite=0.4) - 0.1)

    def test_swing_up_parte_in_basso_e_limite_di_corsa(self):
        env = CartPendoloEnv(configurazione=self.cfg)
        for seed in range(5):
            _, info = env.reset(seed=seed)
            self.assertGreater(abs(info['angle_deg']), 170)
        self.assertAlmostEqual(env.limite_corsa, 0.40)
        self.assertAlmostEqual(env.limite, 0.462)   # scala dell'ingresso invariata
        env.reset(options={'exact': True})
        env.data.qpos[0] = 0.40
        _, r, done, _, info = env.step([1.])
        self.assertTrue(done)
        self.assertEqual(info['termination_reason'], 'finecorsa')
        self.assertLessEqual(r, -self.cfg.episodio.penalita_finecorsa + 1)

    def test_partenza_alto_solo_in_addestramento(self):
        c = replace(self.cfg, episodio=replace(self.cfg.episodio, frazione_partenza_alto=1.))
        alto = CartPendoloEnv(configurazione=c, addestramento=True)
        basso = CartPendoloEnv(configurazione=c)
        self.assertLess(abs(alto.reset(seed=1)[1]['angle_deg']), 10)
        self.assertGreater(abs(basso.reset(seed=1)[1]['angle_deg']), 170)

    def test_successo_swing_up(self):
        env = CartPendoloEnv(configurazione=self.cfg)
        env.reset(seed=0)
        env.steps = env.max_steps
        salita = self.cfg.episodio.tempo_max_salita_s
        env.stable_steps = env.max_steps - int(round(salita / env.dt))   # in alto da tempo_max_salita_s
        self.assertTrue(env.successo(False))
        env.stable_steps -= 1
        self.assertFalse(env.successo(False))
        self.assertFalse(CartPendoloEnv().successo(False))

    def test_guasti_sensori(self):
        p = replace(ParametriDisturbi(), intensita=0.35)
        # Sensori a zero: identico all'ambiente disturbato senza configurazione.
        zero = replace(Configurazione(disturbi=p), sensori=ParametriSensori())
        a, b = crea_ambiente(p), crea_ambiente(p, configurazione=zero)
        for env in (a, b):
            env.reset(seed=3)
        for k in range(100):
            oa = a.step([math.sin(k/7)])[0]
            ob = b.step([math.sin(k/7)])[0]
            np.testing.assert_array_equal(oa, ob)
        # Guasti attivi: offset entro il limite, deriva che cresce col moto.
        forti = replace(self.cfg, sensori=ParametriSensori(
            offset_max_gradi=2., prob_conteggio_perso_max=0.5, salti_al_s=0.,
            passi_persi_max=0.5))
        env = crea_ambiente(p, configurazione=forti)
        env.reset(seed=4)
        self.assertLessEqual(abs(math.degrees(env.errore_encoder_rad)), 2.)
        iniziale = env.errore_encoder_rad
        for k in range(150):
            env.step([1. if (k // 25) % 2 else -1.])
        self.assertNotEqual(env.errore_encoder_rad, iniziale)
        self.assertGreater(abs(env.errore_x_m), 0.)
        # Riproducibile a parita di seed.
        env2 = crea_ambiente(p, configurazione=forti)
        env2.reset(seed=4)
        for k in range(150):
            env2.step([1. if (k // 25) % 2 else -1.])
        self.assertEqual(env.errore_encoder_rad, env2.errore_encoder_rad)

    def test_configurazione_salvata_e_vecchi_checkpoint(self):
        from addestra import contract, firma
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            (path/'config.json').write_text(json.dumps({'environment': contract(ParametriDisturbi())}))
            self.assertIsNone(configurazione_ambiente(path))
            self.assertEqual(configurazione_modello(path).disturbi, ParametriDisturbi())
            self.assertFalse(configurazione_modello(path).episodio.swing_up)
            (path/'config.json').write_text(json.dumps(
                {'firma': firma(self.cfg), 'configurazione': self.cfg.a_dict()}))
            self.assertEqual(configurazione_ambiente(path), self.cfg)


if __name__ == '__main__':
    unittest.main()
