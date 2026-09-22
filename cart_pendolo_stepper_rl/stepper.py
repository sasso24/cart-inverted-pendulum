"""Modello equivalente STEP/DIR, senza simulare ogni impulso elettrico.

La rete decide un'accelerazione; un integratore genera una traiettoria e il
conteggio intero dei microstep. Il servo PD rappresenta SOLO la fisica equivalente
motore/cinghia: NON presuppone un encoder o un PD nel firmware reale.
Non è un modello elettromagnetico e non riproduce risonanze o perdita di sincronismo.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ParametriStepper:
    # TUTTI valori provvisori: sostituire con targa, driver e misure reali.
    passo_cinghia_m: float = 0.002
    denti_puleggia: int = 48
    passi_giro: int = 200
    microstep: int = 8
    velocita_max: float = 3.0           # m/s; non una specifica del motore reale
    accelerazione_max: float = 12.0    # m/s²
    forza_max: float = 20.0            # N; limite equivalente costante
    rigidezza: float = 2000.0          # N/m, inseguimento posizione
    smorzamento: float = 80.0          # N s/m, inseguimento velocità
    errore_massimo: float = 0.05       # m, criterio DIAGNOSTICO solo simulazione

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in self.__dict__.values()):
            raise ValueError('I parametri stepper devono essere positivi e finiti.')
        for value in (self.denti_puleggia, self.passi_giro, self.microstep):
            if int(value) != value:
                raise ValueError('Denti, passi/giro e microstep devono essere interi.')

    @property
    def metri_impulso(self):
        return self.passo_cinghia_m * self.denti_puleggia / (self.passi_giro * self.microstep)


def limita(value, bound):
    """Saturazione scalare: evita array temporanei nel ciclo fisico a 500 Hz."""
    return max(-bound, min(bound, value))


class Stepper:
    def __init__(self, parametri=None):
        self.p = parametri or ParametriStepper()
        self.reset(0.0)

    def reset(self, posizione):
        # L'homing è assunto già concluso. Nessun accesso a GPIO o hardware.
        self.impulsi = round(posizione / self.p.metri_impulso)
        self.posizione_continua = self.posizione
        self.velocita = 0.0

    @property
    def posizione(self):
        return self.impulsi * self.p.metri_impulso

    def avanza(self, azione, dt, x_reale, v_reale):
        """Integra un sottopasso e restituisce la forza equivalente limitata.

        Si conserva la frazione di microstep nell'integratore, altrimenti piccole
        velocità andrebbero perse per arrotondamento. Sul microcontrollore questo
        ruolo spetta al timer/accumulatore di fase; il conteggio deve riflettere
        gli impulsi EMESSI, non il numero di impulsi richiesto alla periferica.
        """
        precedente = self.velocita
        self.velocita = limita(precedente + azione * self.p.accelerazione_max * dt,
                               self.p.velocita_max)
        self.posizione_continua += 0.5 * (precedente + self.velocita) * dt
        self.impulsi = round(self.posizione_continua / self.p.metri_impulso)
        forza = (self.p.rigidezza * (self.posizione - x_reale)
                 + self.p.smorzamento * (self.velocita - v_reale))
        return limita(forza, self.p.forza_max)
