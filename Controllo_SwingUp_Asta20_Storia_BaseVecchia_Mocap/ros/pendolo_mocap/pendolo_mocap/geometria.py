"""Angolo del pendolo dall'orientamento del corpo rigido montato sull'asta.

Convenzione del firmware (cart_pendolo_asta20.xml: cerniera su y, carrello su x,
z in alto): theta = atan2(d . x, d . su), con d = direzione dal perno alla punta
del braccio lungo. 0 = in alto, +/-pi = appeso, positivo verso +x.

d nel sistema del corpo rigido si ricava una volta con il pendolo fermo appeso
(d = -su): non serve sapere come e' stato definito il corpo rigido nel software
del tracciamento, e la rotazione attorno all'asta non conta. La posizione del
carrello non entra: basta l'orientamento dell'asta.
Nessuna dipendenza da ROS.
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

Vettore = tuple[float, float, float]
Quaternione = tuple[float, float, float, float]   # x, y, z, w (come geometry_msgs)


def _norma(v: Sequence[float]) -> float:
    return math.sqrt(sum(c*c for c in v))


def normalizza(v: Sequence[float]) -> Vettore:
    n = _norma(v)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError(f'vettore nullo o non finito: {tuple(v)}')
    return (v[0]/n, v[1]/n, v[2]/n)


def _scalare(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def _vettoriale(a: Sequence[float], b: Sequence[float]) -> Vettore:
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])


def ruota(q: Quaternione, v: Sequence[float]) -> Vettore:
    """v dal sistema del corpo al sistema del tracciamento (q normalizzato)."""
    x, y, z, w = q
    u = (x, y, z)
    t = _vettoriale(u, v)
    t = (2*t[0], 2*t[1], 2*t[2])
    c = _vettoriale(u, t)
    return (v[0] + w*t[0] + c[0], v[1] + w*t[1] + c[1], v[2] + w*t[2] + c[2])


def ruota_inversa(q: Quaternione, v: Sequence[float]) -> Vettore:
    x, y, z, w = q
    return ruota((-x, -y, -z, w), v)


def normalizza_quaternione(q: Sequence[float]) -> Quaternione:
    n = _norma(q)
    if not math.isfinite(n) or n < 1e-6:
        raise ValueError(f'quaternione non valido: {tuple(q)}')
    return (q[0]/n, q[1]/n, q[2]/n, q[3]/n)


class AngoloPendolo:
    """asse_su: verticale verso l'alto nel sistema del tracciamento (z per ROS,
    spesso y per Motive/OptiTrack senza conversione). asse_carrello: direzione
    del binario verso il finecorsa destro (guardando da dietro): +x del firmware."""

    def __init__(self, asse_su: Sequence[float], asse_carrello: Sequence[float],
                 direzione_corpo: Sequence[float] | None = None) -> None:
        self.su = normalizza(asse_su)
        x = tuple(a - _scalare(asse_carrello, self.su)*s for a, s in zip(asse_carrello, self.su))
        if _norma(x) < 0.5*_norma(asse_carrello):
            raise ValueError('asse_carrello quasi parallelo a asse_su')
        self.x = normalizza(x)
        self.direzione_corpo = normalizza(direzione_corpo) if direzione_corpo else None

    @property
    def calibrato(self) -> bool:
        return self.direzione_corpo is not None

    def calibra(self, quaternioni: Iterable[Sequence[float]]) -> Vettore:
        """Pendolo fermo appeso: la direzione 'giu' nel sistema del corpo e' d."""
        giu = (-self.su[0], -self.su[1], -self.su[2])
        somma = [0.0, 0.0, 0.0]
        n = 0
        for q in quaternioni:
            d = ruota_inversa(normalizza_quaternione(q), giu)
            somma = [s + c for s, c in zip(somma, d)]
            n += 1
        if n == 0:
            raise ValueError('nessun campione per la calibrazione')
        self.direzione_corpo = normalizza(somma)
        return self.direzione_corpo

    def direzione(self, q: Sequence[float]) -> Vettore:
        if self.direzione_corpo is None:
            raise RuntimeError('direzione dell\'asta non calibrata')
        return ruota(normalizza_quaternione(q), self.direzione_corpo)

    def angolo(self, q: Sequence[float]) -> float:
        d = self.direzione(q)
        return math.atan2(_scalare(d, self.x), _scalare(d, self.su))

    def scostamento_da_appeso(self, q: Sequence[float]) -> float:
        """Angolo (rad) fra l'asta e la verticale verso il basso, in ogni direzione."""
        d = self.direzione(q)
        return math.acos(max(-1.0, min(1.0, -_scalare(d, self.su))))

    def fuori_piano(self, q: Sequence[float]) -> float:
        """Angolo (rad) fra l'asta e il piano di oscillazione: deve restare
        piccolo. Grande = asse_carrello o asse_su sbagliati."""
        d = self.direzione(q)
        normale = _vettoriale(self.su, self.x)
        return math.asin(max(-1.0, min(1.0, _scalare(d, normale))))
