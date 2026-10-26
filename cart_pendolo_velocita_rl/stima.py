"""Stimatore dello stato dalle misure rumorose: lo stesso algoritmo va sul firmware.

Perche serve: derivando misure rumorose a ogni periodo (20 o 10 ms) il rumore esplode.
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
  e a ogni periodo della rete, con la lettura dell'encoder:
      r = wrap(theta_misurato - theta) ; theta += L_theta*r ; omega += L_omega*r

Sul firmware: `predici` nell'interrupt della rampa a 500 Hz, `correggi`
nel ciclo della rete (50 o 100 Hz), prima di preparare i sei ingressi.
I guadagni L sono per lettura: cambiando il periodo vanno ricalcolati
(guadagni_equivalenti) per avere la stessa dinamica dell'errore.
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
    # Offset dell'encoder stimato come terzo stato (0 = stimatore storico a due
    # stati). L'offset e osservabile perche la gravita lega l'accelerazione
    # angolare all'angolo vero: un angolo letto spostato di b produce una
    # caduta prevista che le letture successive non confermano. l_b < 0
    # nell'assegnamento dei poli (guadagni_con_offset); |b| <= b_max_rad.
    # L'offset si aggiorna con peso max(cos(theta), 0): solo nella meta alta.
    l_b: float = 0.0
    b_max_rad: float = 0.2617993877991494   # 15 gradi

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value) or (value < 0 and name != 'l_b'):
                raise ValueError(f'{name}: serve un numero finito non negativo.')
        if not -1 < self.l_b <= 0:
            raise ValueError('Serve -1 < l_b <= 0.')
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
    # L'attrito coulombiano (frictionloss) entrerebbe in mj_inverse e falserebbe k_g.
    attrito = model.dof_frictionloss.copy()
    model.dof_frictionloss[:] = 0.

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
    model.dof_frictionloss[:] = attrito
    return {'k_g': gravita, 'k_a': accoppiamento / inerzia,
            'k_d': float(model.dof_damping[1]) / inerzia}


class StimatoreStato:
    def __init__(self, parametri=None):
        self.p = parametri if parametri is not None else ParametriStimatore.carica()
        self.reset(0., 0.)

    def reset(self, x, theta, velocita=0., omega=0.):
        """Prima lettura: posizione e angolo misurati, pendolo fermo (omega=0 come
        nel firmware; un valore diverso serve solo alle partenze casuali in addestramento)."""
        self.x, self.theta, self.omega = float(x), float(theta), float(omega)
        self.offset = 0.
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
        """Una lettura dei sensori a ogni periodo della rete."""
        p = self.p
        r = math.remainder(theta_misurato - self.theta - self.offset, 2*math.pi)
        if p.l_b:
            # Solo nella meta alta: in basso la gravita cambia segno e con lo
            # stesso guadagno l'offset divergerebbe (in basso il firmware ricalibra).
            peso = max(math.cos(self.theta), 0.)
            self.offset = max(-p.b_max_rad, min(p.b_max_rad, self.offset + p.l_b*peso*r))
        self.theta = math.remainder(self.theta + p.l_theta*r, 2*math.pi)
        self.omega += p.l_omega*r
        self.x += p.l_x*(x_misurata - self.x)


def guadagni_equivalenti(p, periodo_da, periodo_a, dt=0.002):
    """l_theta, l_omega, l_x per il periodo `periodo_a` con la stessa dinamica
    dell'errore (poli in s, linearizzata in alto) dei guadagni p a `periodo_da`."""
    import numpy as np
    from scipy.optimize import least_squares

    def poli(lt, lo, n):
        a = np.eye(2)
        passo = np.array([[1., dt], [0., 1.]]) @ np.array([[1., 0.], [dt*p.k_g, 1. - dt*p.k_d]])
        for _ in range(n):
            a = passo @ a
        lam = np.linalg.eigvals(np.array([[1. - lt, 0.], [-lo, 1.]]) @ a).astype(complex)
        return np.sort_complex(np.log(lam) / (n*dt))

    n_da, n_a = round(periodo_da/dt), round(periodo_a/dt)
    obiettivo = poli(p.l_theta, p.l_omega, n_da)

    def errore(v):
        d = poli(v[0], v[1], n_a) - obiettivo
        return np.r_[d.real, d.imag]
    scala = periodo_a / periodo_da
    lt, lo = least_squares(errore, [p.l_theta*scala, p.l_omega*scala]).x
    l_x = 1. - (1. - p.l_x)**scala
    return {'l_theta': float(lt), 'l_omega': float(lo), 'l_x': float(l_x)}


def guadagni_con_offset(p, periodo, polo_offset_s=-1.0, riferimento=(0.2, 1.0, 0.02), dt=0.002):
    """l_theta, l_omega, l_b dello stimatore a tre stati (theta, omega, offset).

    Assegnamento dei poli sul modello linearizzato in alto: i due poli
    dell'errore restano quelli dei guadagni `riferimento` (l_theta, l_omega,
    periodo) dello stimatore storico, il terzo (offset) e `polo_offset_s` in 1/s.
    """
    import numpy as np
    from scipy.signal import place_poles

    def phi(n):
        a = np.eye(2)
        passo = np.array([[1., dt], [0., 1.]]) @ np.array([[1., 0.], [dt*p.k_g, 1. - dt*p.k_d]])
        for _ in range(n):
            a = passo @ a
        return a
    lt, lo, periodo_rif = riferimento
    n_rif = round(periodo_rif/dt)
    lam = np.linalg.eigvals(np.array([[1. - lt, 0.], [-lo, 1.]]) @ phi(n_rif)).astype(complex)
    poli_s = np.log(lam) / (n_rif*dt)
    n = round(periodo/dt)
    f = np.eye(3)
    f[:2, :2] = phi(n)
    h = np.array([[1., 0., 1.]])
    z = np.exp(np.r_[poli_s, polo_offset_s] * n*dt)
    l = place_poles(f.T, (h @ f).T, z).gain_matrix.ravel()
    return {'l_theta': float(l[0]), 'l_omega': float(l[1]), 'l_b': float(l[2])}
