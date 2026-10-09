"""Stimatore dello stato dalle misure rumorose: lo stesso algoritmo va sul firmware.

Perche serve: derivando misure rumorose ogni 20 ms il rumore esplode.
Con 7,5 mm di rumore di posizione la velocita derivata ha circa 0,53 m/s
di rumore (il carrello arriva a 0,48 m/s); con 0,75 gradi sull'angolo la
velocita angolare derivata ha circa 0,9 rad/s di rumore. La rete vedeva
quasi solo rumore e non riusciva piu a stabilizzare il pendolo.

Idea: prevedere lo stato con la fisica nota e correggerlo poco con le misure.

- Carrello: la posizione avanza con la velocita applicata dalla rampa
  (sul banco: il conteggio degli impulsi STEP emessi) e viene corretta
  lentamente dalla misura di posizione. La velocita e quella applicata.
- Pendolo, a ogni passo della rampa (2 ms):
      alfa  = K_G*sin(theta) - K_A*cos(theta)*a_carrello - K_D*omega
      omega += dt*alfa ;  theta += dt*omega
  e ogni 20 ms, con la lettura dell'encoder:
      r = wrap(theta_misurato - theta) ; theta += L_theta*r ; omega += L_omega*r

Sul firmware: `predici` nell'interrupt della rampa a 500 Hz, `correggi`
nel ciclo della rete a 50 Hz, prima di preparare i sei ingressi.
Il modello e non lineare in theta: vale anche lontano dalla verticale.
"""
from dataclasses import dataclass, asdict
import math
from pathlib import Path
import tomllib

import mujoco

CONFIG_DEFAULT = Path(__file__).resolve().parent / 'stimatore.toml'


@dataclass(frozen=True)
class ParametriStimatore:
    k_g: float = 17.1317      # 1/s^2, m*l*g/J del pendolo
    k_a: float = 1.74635      # rad/m, accoppiamento con l'accelerazione del carrello
    k_d: float = 0.35034      # 1/s, smorzamento del giunto / J
    l_theta: float = 0.2      # correzione dell'angolo per lettura encoder
    l_omega: float = 1.0      # 1/s, correzione della velocita angolare
    l_x: float = 0.05         # correzione della posizione per lettura

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or value < 0:
                raise ValueError(f'{name}: serve un numero finito non negativo.')
        if not 0 < self.l_theta < 1 or not 0 <= self.l_x <= 1:
            raise ValueError('Serve 0 < l_theta < 1 e 0 <= l_x <= 1.')

    @classmethod
    def carica(cls, path=CONFIG_DEFAULT):
        with Path(path).open('rb') as stream:
            config = tomllib.load(stream)
        if set(config) != {'stimatore'}:
            raise ValueError('Serve solo la sezione [stimatore].')
        return cls(**config['stimatore'])


def coefficienti_da_modello(model):
    """K_G, K_A, K_D del modello MuJoCo, per controllare il TOML dopo modifiche all'XML."""
    data = mujoco.MjData(model)

    def forza(theta, qacc):
        mujoco.mj_resetData(model, data)
        data.qpos[:] = [0., theta]
        data.qacc[:] = qacc
        mujoco.mj_inverse(model, data)
        return float(data.qfrc_inverse[1])

    zero = forza(0., [0., 0.])
    inerzia = forza(0., [0., 1.]) - zero
    accoppiamento = forza(0., [1., 0.]) - zero
    theta = 0.3
    gravita = -forza(theta, [0., 0.]) / (inerzia * math.sin(theta))
    return {'k_g': gravita, 'k_a': accoppiamento / inerzia,
            'k_d': float(model.dof_damping[1]) / inerzia}


class StimatoreStato:
    def __init__(self, parametri=None):
        self.p = parametri if parametri is not None else ParametriStimatore.carica()
        self.reset(0., 0.)

    def reset(self, x, theta, velocita=0.):
        """Prima lettura: posizione e angolo misurati, pendolo considerato fermo."""
        self.x, self.theta, self.omega = float(x), float(theta), 0.
        self.velocita = self._velocita_precedente = float(velocita)

    def predici(self, velocita_applicata, dt):
        """Un passo della rampa: velocita del carrello in m/s dopo la rampa."""
        p = self.p
        accelerazione = (velocita_applicata - self._velocita_precedente) / dt
        self._velocita_precedente = velocita_applicata
        alfa = (p.k_g*math.sin(self.theta) - p.k_a*math.cos(self.theta)*accelerazione
                - p.k_d*self.omega)
        self.omega += dt*alfa
        self.theta += dt*self.omega
        self.x += dt*velocita_applicata
        self.velocita = velocita_applicata

    def correggi(self, x_misurata, theta_misurato):
        """Una lettura dei sensori ogni 20 ms."""
        p = self.p
        r = math.remainder(theta_misurato - self.theta, 2*math.pi)
        self.theta = math.remainder(self.theta + p.l_theta*r, 2*math.pi)
        self.omega += p.l_omega*r
        self.x += p.l_x*(x_misurata - self.x)
