"""Novita del 16/10: periodo di controllo 10 ms, partenze casuali, inversione
con arresto (stepper.py = firmware), guadagni equivalenti dello stimatore."""
from dataclasses import replace
import math
import unittest

import numpy as np

from ambiente import CartPendoloEnv, ROOT
from configurazione import Configurazione
from disturbi import crea_ambiente
from stepper import ControlloStepper, ParametriStepper
import stima


class PeriodoTest(unittest.TestCase):
    def setUp(self):
        self.cfg = Configurazione.carica(ROOT / 'config.toml')

    def _con(self, **episodio):
        return replace(self.cfg, episodio=replace(self.cfg.episodio, **episodio))

    def test_periodo_10_ms(self):
        env = CartPendoloEnv(configurazione=self._con(periodo_controllo_s=0.01))
        self.assertEqual(env.frame_skip, 5)
        self.assertAlmostEqual(env.dt, 0.01)
        self.assertEqual(env.max_steps, 2000)
        self.assertEqual(env.scala_ricompensa, 0.5)
        # Il periodo storico resta identico (10 passi, nessuna scala).
        env = CartPendoloEnv(configurazione=self._con(periodo_controllo_s=0.02))
        self.assertEqual((env.frame_skip, env.max_steps, env.scala_ricompensa), (10, 1000, 1.))
        self.assertEqual(CartPendoloEnv().frame_skip, 10)
        with self.assertRaises(ValueError):
            CartPendoloEnv(configurazione=self._con(periodo_controllo_s=0.005))

    def test_ricompensa_al_secondo_invariata(self):
        """Stesso stato tenuto fermo per 1 s: stessa ricompensa a 50 e 100 Hz."""
        totali = []
        for periodo in (0.02, 0.01):
            env = CartPendoloEnv(configurazione=self._con(periodo_controllo_s=periodo, swing_up=False))
            env.reset(seed=0, options={'exact': True})
            totali.append(sum(env._ricompensa(0.) * env.scala_ricompensa
                              for _ in range(round(1 / env.dt))))
        self.assertAlmostEqual(totali[0], totali[1], places=9)

    def test_partenze_casuali_solo_in_addestramento(self):
        c = self._con(frazione_partenza_alto=0., frazione_partenza_casuale=1.)
        for disturbi in (None, c.disturbi):
            env = crea_ambiente(disturbi, configurazione=c, addestramento=True)
            angoli, velocita = [], []
            for seed in range(40):
                _, info = env.reset(seed=seed)
                self.assertTrue(env.partenza_casuale)
                angoli.append(info['angle_deg'])
                velocita.append(info['cart_velocity_m_s'])
                self.assertLessEqual(abs(info['cart_x']), c.episodio.casuale_x_m)
                # Rampa e servo partono dalla velocita del carrello.
                self.assertAlmostEqual(env.comando_motore.velocita_carrello_m_s,
                                       info['cart_velocity_m_s'], places=9)
            self.assertGreater(max(angoli) - min(angoli), 270)
            self.assertGreater(max(map(abs, velocita)), 0.15)
            env.step(np.array([0.], dtype=np.float32))
        valuta = crea_ambiente(c.disturbi, configurazione=c)
        for seed in range(10):
            _, info = valuta.reset(seed=seed)
            self.assertFalse(valuta.partenza_casuale)
            self.assertGreater(abs(info['angle_deg']), 170)

    def test_inversione_ferma_a_zero(self):
        p = replace(ParametriStepper.carica(), tick_fermo_inversione=1)
        c = ControlloStepper(p)
        c.imposta(1800.)
        self.assertEqual([round(c.avanza(-0.1, 0.002).frequenza_hz) for _ in range(3)],
                         [0, -3200, -6080])
        # Arrivo esatto a zero: nessun periodo in piu.
        c.imposta(3200.)
        self.assertEqual([round(c.avanza(-1., 0.002).frequenza_hz) for _ in range(2)], [0, -3200])
        # Due periodi fermi.
        c = ControlloStepper(replace(p, tick_fermo_inversione=2))
        c.imposta(1800.)
        self.assertEqual([round(c.avanza(-1., 0.002).frequenza_hz) for _ in range(3)], [0, 0, -3200])
        # Storico: attraversa lo zero.
        c = ControlloStepper(replace(p, tick_fermo_inversione=0))
        c.imposta(1800.)
        self.assertEqual(round(c.avanza(-1., 0.002).frequenza_hz), -1400)

    def test_guadagni_equivalenti(self):
        p = stima.ParametriStimatore.carica()
        g = stima.guadagni_equivalenti(p, 0.02, 0.01)
        self.assertAlmostEqual(g['l_theta'], 0.105573, places=5)
        self.assertAlmostEqual(g['l_omega'], 0.528152, places=5)
        self.assertAlmostEqual(g['l_x'], 0.025321, places=5)
        self.assertAlmostEqual(stima.guadagni_equivalenti(p, 0.02, 0.02)['l_theta'], p.l_theta)
        self.assertTrue(math.isclose((1 - g['l_x'])**2, 1 - p.l_x))


class OffsetTest(unittest.TestCase):
    """Stimatore a tre stati: guadagni, compatibilita e stima di un offset vero."""

    def test_guadagni_e_config(self):
        p = stima.ParametriStimatore.carica()
        st = Configurazione.carica(ROOT / 'config.toml').stimatore
        g = stima.guadagni_con_offset(p, 0.02)
        for nome in ('l_theta', 'l_omega', 'l_b'):
            self.assertAlmostEqual(g[nome], getattr(st, nome), places=5)
        g10 = stima.guadagni_con_offset(p, 0.01)
        self.assertAlmostEqual(g10['l_b'], -0.023486, places=5)

    def test_senza_offset_identico(self):
        a = stima.StimatoreStato(stima.ParametriStimatore.carica())
        b = stima.StimatoreStato(replace(stima.ParametriStimatore.carica(), l_b=0.))
        rng = np.random.default_rng(0)
        for s in (a, b):
            s.reset(0., 3.0)
        for _ in range(200):
            v, m = rng.normal(0, 0.1), rng.uniform(-3, 3)
            for s in (a, b):
                s.predici(v, 0.002)
                s.correggi(0., m)
        self.assertEqual((a.theta, a.omega, a.offset), (b.theta, b.omega, b.offset))

    def test_offset_stimato_in_equilibrio(self):
        """Rete del 14/10 + stimatore con offset, encoder spostato di 4 gradi:
        dopo lo swing-up l'offset stimato si avvicina a quello vero."""
        from stable_baselines3 import SAC
        from configurazione import configurazione_modello
        cartella = ROOT / 'modelli_swingup'
        if not (cartella / 'migliore.zip').exists():
            self.skipTest('modelli_swingup non disponibile')
        cfg = configurazione_modello(cartella)
        g = stima.guadagni_con_offset(cfg.stimatore, 0.02)
        cfg = replace(cfg, stimatore=replace(cfg.stimatore, **g),
                      sensori=replace(cfg.sensori, prob_conteggio_perso_max=0., salti_al_s=0.))
        agente = SAC.load(cartella / 'migliore.zip', device='cpu')
        env = crea_ambiente(cfg.disturbi, configurazione=cfg)
        obs, _ = env.reset(seed=2001)
        env.errore_encoder_rad = math.radians(4.)
        env._misura(iniziale=True)
        obs = env.observation()
        for _ in range(env.max_steps):
            obs, _, fine, tronco, _ = env.step(agente.predict(obs, deterministic=True)[0])
            if fine or tronco:
                break
        self.assertTrue(env.successo(fine))
        vero = env.errore_encoder_rad + env._offset
        self.assertLess(abs(env.stimatore.offset - vero), math.radians(1.))


if __name__ == '__main__':
    unittest.main()
