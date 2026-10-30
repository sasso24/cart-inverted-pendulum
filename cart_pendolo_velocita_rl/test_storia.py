"""Novita del 23/10: storia degli ingressi passati ([storia] in config.toml, storia.py)."""
from dataclasses import replace
import unittest

import numpy as np

from ambiente import ROOT
from configurazione import Configurazione, ParametriStoria
from disturbi import crea_ambiente

RITARDI = (1, 2, 3, 5, 10, 25)


class StoriaTest(unittest.TestCase):
    def setUp(self):
        # config.toml ha gia una storia: qui serve la base senza ritardi.
        self.cfg = replace(Configurazione.carica(ROOT / 'config.toml'), storia=ParametriStoria())
        self.cfg_storia = replace(self.cfg, storia=ParametriStoria(RITARDI))

    def _percorso(self, env, passi=60, seed=3):
        """Osservazioni, ingressi attuali e azioni con comandi casuali ripetibili."""
        rng = np.random.default_rng(seed)
        obs, _ = env.reset(seed=seed)
        osservazioni, attuali, azioni = [obs], [env.osservazione_attuale()
                                                if hasattr(env, 'osservazione_attuale') else obs], []
        for _ in range(passi):
            u = float(rng.uniform(-1.3, 1.3))
            obs, *_ = env.step(np.array([u], np.float32))
            azioni.append(float(np.clip(u, -1., 1.)))
            osservazioni.append(obs)
            attuali.append(env.osservazione_attuale() if hasattr(env, 'osservazione_attuale') else obs)
        return osservazioni, attuali, azioni

    def test_ingressi_e_azioni_ai_ritardi(self):
        for disturbi in (None, self.cfg.disturbi):
            env = crea_ambiente(disturbi, configurazione=self.cfg_storia)
            self.assertEqual(env.observation_space.shape, (6 + 7*len(RITARDI),))
            osservazioni, attuali, azioni = self._percorso(env)
            for t, obs in enumerate(osservazioni):
                self.assertEqual(obs.shape, (6 + 7*len(RITARDI),))
                np.testing.assert_array_equal(obs[:6], attuali[t])
                for j, k in enumerate(RITARDI):
                    blocco = obs[6 + 7*j: 13 + 7*j]
                    # Prima dell'inizio: i primi ingressi e u = 0 (come il firmware).
                    atteso = attuali[max(t - k, 0)]
                    u = azioni[t - k] if t - k >= 0 else 0.
                    np.testing.assert_array_equal(blocco[:6], atteso)
                    self.assertAlmostEqual(float(blocco[6]), u, places=6)
            # observation() non cambia la storia.
            np.testing.assert_array_equal(env.observation(), env.observation())

    def test_reset_azzera_la_storia(self):
        env = crea_ambiente(self.cfg.disturbi, configurazione=self.cfg_storia)
        self._percorso(env, passi=20)
        obs, _ = env.reset(seed=11)
        for j in range(len(RITARDI)):
            np.testing.assert_array_equal(obs[6 + 7*j: 12 + 7*j], obs[:6])
            self.assertEqual(obs[12 + 7*j], 0.)

    def test_senza_storia_identico(self):
        """Senza ritardi: stessa classe, stessi sei ingressi e stessa traiettoria della storia
        (la storia non consuma numeri casuali e non cambia la fisica)."""
        for disturbi in (None, self.cfg.disturbi):
            base = crea_ambiente(disturbi, configurazione=self.cfg)
            self.assertFalse(hasattr(base, 'osservazione_attuale'))
            self.assertEqual(base.observation_space.shape, (6,))
            senza, _, _ = self._percorso(base)
            _, con, _ = self._percorso(crea_ambiente(disturbi, configurazione=self.cfg_storia))
            for a, b in zip(senza, con):
                np.testing.assert_array_equal(a, b)

    def test_configurazione(self):
        self.assertEqual(ParametriStoria().ingressi, 6)
        self.assertEqual(ParametriStoria([1, 2, 15]).ritardi, (1, 2, 15))
        for sbagliati in ([0, 1], [2, 1], [1, 1], [1.5]):
            with self.assertRaises(ValueError):
                ParametriStoria(sbagliati)
        dati = self.cfg_storia.a_dict()
        self.assertEqual(Configurazione.da_dict(dati).storia.ritardi, RITARDI)
        del dati['storia']
        self.assertEqual(Configurazione.da_dict(dati).storia.ritardi, ())


if __name__ == '__main__':
    unittest.main()
