"""Protocollo seriale fra i nodi ROS e il firmware Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap.

USART2 della Nucleo (ST-LINK VCP, /dev/ttyACM*), 115200 8N1, little-endian.
Stesse strutture di Core/Src/main.c (MocapAngolo, MocapTelemetria).

PC -> scheda, 11 byte, uno per campione del tracciamento (200 Hz):
    A5 5A | tipo 'M' u8 | stato u8 | seq u16 | theta f32 | checksum
    stato bit 0 = corpo rigido tracciato; theta in rad, convenzione del firmware:
    0 = braccio lungo in alto, +/-pi = appeso, positivo con la punta verso +x
    (verso il finecorsa destro, guardando il banco da dietro).

scheda -> PC, 37 byte, uno per periodo della rete (20 ms):
    A5 5A | tipo 'T' | stato | errore | eta_ms | seq u16 | ricevuti u16 | scartati u16
          | tick_ms u32 | theta, theta_encoder, stima_theta, stima_x, azione f32 | checksum

checksum = complemento a due della somma dei byte del corpo (come l'HIL).
Nessuna dipendenza da ROS: lo usano anche i test del firmware.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import struct

SYNC = b'\xa5\x5a'
TIPO_ANGOLO = 0x4D
TIPO_TELEMETRIA = 0x54
STATO_TRACCIATO = 0x01

_ANGOLO = struct.Struct('<BBHf')
_TELEMETRIA = struct.Struct('<BBBBHHHIfffff')
LUNGHEZZA_ANGOLO = len(SYNC) + _ANGOLO.size + 1
LUNGHEZZA_TELEMETRIA = len(SYNC) + _TELEMETRIA.size + 1

# controllo_errore del firmware (LEGGIMI.md, MOCAP.md).
CODICI_ERRORE = {
    0: 'nessun errore',
    1: 'inizializzazione della rete',
    2: 'inferenza della rete',
    3: 'NaN/Inf',
    4: 'finecorsa o limite di corsa',
    5: 'periodo scaduto',
    6: 'avvio PWM',
    7: 'riferimento dell\'angolo iniziale',
    9: 'salto dell\'angolo',
    10: 'angolo oltre il limite (solo equilibrio)',
    11: 'avvio rifiutato: pendolo non fermo o non in posizione',
    12: 'angolo dal tracciamento assente o scaduto',
}


def checksum(corpo: bytes) -> int:
    return (-sum(corpo)) & 0xFF


def pacchetto_angolo(theta: float, seq: int, tracciato: bool = True) -> bytes:
    corpo = _ANGOLO.pack(TIPO_ANGOLO, STATO_TRACCIATO if tracciato else 0, seq & 0xFFFF, theta)
    return SYNC + corpo + bytes([checksum(corpo)])


def avvolgi(angolo: float) -> float:
    """Angolo in (-pi, pi], come Stima_Wrap del firmware."""
    angolo = math.remainder(angolo, 2.0*math.pi)
    return angolo + 2.0*math.pi if angolo <= -math.pi else angolo


@dataclass(frozen=True)
class Telemetria:
    stato: int
    errore: int
    eta_ms: int
    seq: int
    ricevuti: int
    scartati: int
    tick_ms: int
    theta: float
    theta_encoder: float
    stima_theta: float
    stima_x: float
    azione: float

    @property
    def controllo_attivo(self) -> bool:
        return bool(self.stato & 1)

    @property
    def angolo_valido(self) -> bool:
        return bool(self.stato & 2)

    @property
    def theta_inizializzato(self) -> bool:
        return bool(self.stato & 4)

    @property
    def carrello_inizializzato(self) -> bool:
        return bool(self.stato & 8)

    @property
    def descrizione_errore(self) -> str:
        return CODICI_ERRORE.get(self.errore, f'codice {self.errore}')


def pacchetto_telemetria(t: Telemetria) -> bytes:
    """Come Telemetria_Invia del firmware (per i test)."""
    corpo = _TELEMETRIA.pack(TIPO_TELEMETRIA, t.stato, t.errore, t.eta_ms, t.seq, t.ricevuti,
                             t.scartati, t.tick_ms, t.theta, t.theta_encoder, t.stima_theta,
                             t.stima_x, t.azione)
    return SYNC + corpo + bytes([checksum(corpo)])


class LettoreTelemetria:
    """Ricostruisce i pacchetti di telemetria da byte che arrivano a pezzi."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.scartati = 0

    def aggiungi(self, dati: bytes) -> list[Telemetria]:
        self._buffer += dati
        pacchetti = []
        while True:
            inizio = self._buffer.find(SYNC)
            if inizio < 0:
                # Tiene l'ultimo byte: puo' essere il primo della sincronia.
                del self._buffer[:max(0, len(self._buffer) - 1)]
                return pacchetti
            del self._buffer[:inizio]
            if len(self._buffer) < LUNGHEZZA_TELEMETRIA:
                return pacchetti
            corpo = bytes(self._buffer[2:LUNGHEZZA_TELEMETRIA - 1])
            if corpo[0] != TIPO_TELEMETRIA or checksum(corpo) != self._buffer[LUNGHEZZA_TELEMETRIA - 1]:
                self.scartati += 1
                del self._buffer[:1]   # sincronia falsa: cerca la prossima
                continue
            del self._buffer[:LUNGHEZZA_TELEMETRIA]
            pacchetti.append(Telemetria(*_TELEMETRIA.unpack(corpo)[1:]))
