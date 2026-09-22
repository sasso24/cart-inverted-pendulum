"""Conversione stateless: uscita rete [-1, 1] -> frequenza STEP firmata.

Non simula bobine, coppia, rampe, passi persi o un generatore di impulsi.
Questo modulo usa solo la libreria standard, anche senza MuJoCo.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import tomllib

CONFIG_DEFAULT = Path(__file__).with_name('stepper.toml')


@dataclass(frozen=True)
class ParametriStepper:
    frequenza_min_hz: float
    frequenza_max_hz: float
    passi_per_giro: int
    angolo_passo_gradi: float
    microstepping: int
    passo_cinghia_m: float
    denti_puleggia: int
    rapporto_trasmissione: float = 1.0
    zona_morta: float = 0.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name}: serve un numero finito.')
        if not 0 <= self.frequenza_min_hz < self.frequenza_max_hz:
            raise ValueError('Frequenze: serve 0 <= min < max.')
        if not 0 <= self.zona_morta < 1:
            raise ValueError('zona_morta deve essere in [0, 1).')
        for name in ('passi_per_giro', 'microstepping', 'denti_puleggia'):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f'{name}: serve un intero positivo.')
        for name in ('angolo_passo_gradi', 'passo_cinghia_m', 'rapporto_trasmissione'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name}: deve essere positivo.')
        if not math.isclose(self.passi_per_giro * self.angolo_passo_gradi, 360., rel_tol=1e-6):
            raise ValueError('passi_per_giro e angolo_passo_gradi sono incoerenti.')

    @classmethod
    def carica(cls, percorso=CONFIG_DEFAULT):
        with Path(percorso).open('rb') as stream:
            config = tomllib.load(stream)
        if set(config) != {'stepper'}:
            raise ValueError('La configurazione deve contenere solo la sezione [stepper].')
        return cls(**config['stepper'])

    @property
    def impulsi_per_giro(self):
        return self.passi_per_giro * self.microstepping

    @property
    def metri_per_impulso(self):
        return (self.passo_cinghia_m * self.denti_puleggia
                / (self.impulsi_per_giro * self.rapporto_trasmissione))

    @property
    def velocita_max_m_s(self):
        return self.frequenza_max_hz * self.metri_per_impulso


@dataclass(frozen=True)
class ComandoMotore:
    frequenza_hz: float # firmata: positiva verso +X, negativa verso -X
    velocita_carrello_m_s: float

    @property
    def direzione(self):
        return (self.frequenza_hz > 0) - (self.frequenza_hz < 0)

    @property
    def frequenza_step_hz(self):
        """Modulo non negativo da passare al generatore STEP del firmware."""
        return abs(self.frequenza_hz)


class ControlloStepper:
    def __init__(self, parametri=None):
        self.p = parametri if parametri is not None else ParametriStepper.carica()

    def azione_a_hz(self, azione):
        u = float(azione)
        if not math.isfinite(u):
            raise ValueError('L’azione deve essere finita.')
        u = max(-1., min(1., u))
        if abs(u) <= self.p.zona_morta:
            return 0.
        frazione = (abs(u) - self.p.zona_morta) / (1. - self.p.zona_morta)
        hz = self.p.frequenza_min_hz + frazione * (self.p.frequenza_max_hz - self.p.frequenza_min_hz)
        return math.copysign(hz, u)

    def comando(self, azione):
        hz = self.azione_a_hz(azione)
        return ComandoMotore(hz, hz * self.p.metri_per_impulso)
