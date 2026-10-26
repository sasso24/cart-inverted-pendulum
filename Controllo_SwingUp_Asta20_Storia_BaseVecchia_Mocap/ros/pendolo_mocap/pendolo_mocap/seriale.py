"""Collegamento seriale con la Nucleo: invio degli angoli, lettura della telemetria."""
from __future__ import annotations

import threading
import time
from typing import Callable

from . import protocollo


class Collegamento:
    """Apre la porta, manda gli angoli con seq crescente e chiama
    al_pacchetto(telemetria, ritardo_s) per ogni pacchetto della scheda, dal
    thread di lettura. ritardo_s = da quando e' partito l'angolo che la scheda
    dice di aver usato (seq) a quando e' arrivata la telemetria: comprende
    l'attesa del periodo della rete, quindi e' 0-25 ms anche con una linea
    perfetta. None se il seq non e' fra quelli mandati da questo processo."""

    def __init__(self, porta: str, baud: int,
                 al_pacchetto: Callable[[protocollo.Telemetria, float | None], None]) -> None:
        import serial   # pyserial: python3-serial o pip install pyserial
        self._seriale = serial.Serial(porta, baud, timeout=0.05, write_timeout=0.05)
        self._seriale.reset_input_buffer()
        self._al_pacchetto = al_pacchetto
        self._lettore = protocollo.LettoreTelemetria()
        self._inviati = [0.0] * 65536   # istante di invio per seq
        self._scrittura = threading.Lock()
        self._fine = threading.Event()
        self.seq = 0
        self.inviati = 0
        self.errori_scrittura = 0
        self._thread = threading.Thread(target=self._leggi, daemon=True)
        self._thread.start()

    @property
    def telemetria_scartata(self) -> int:
        return self._lettore.scartati

    def invia_angolo(self, theta: float, tracciato: bool = True) -> int:
        with self._scrittura:
            self.seq = (self.seq + 1) & 0xFFFF
            seq = self.seq
            self._inviati[seq] = time.monotonic()
            self._scrivi(protocollo.pacchetto_angolo(theta, seq, tracciato))
        return seq

    def invia_byte(self, dati: bytes) -> None:
        """Byte qualsiasi (prove di pacchetti errati)."""
        with self._scrittura:
            self._scrivi(dati)

    def _scrivi(self, dati: bytes) -> None:
        import serial
        try:
            self._seriale.write(dati)
            self.inviati += 1
        except serial.SerialTimeoutException:
            self.errori_scrittura += 1

    def _leggi(self) -> None:
        while not self._fine.is_set():
            try:
                dati = self._seriale.read(max(1, self._seriale.in_waiting))
            except Exception:   # porta chiusa o scheda scollegata
                if self._fine.is_set():
                    return
                time.sleep(0.1)
                continue
            if not dati:
                continue
            adesso = time.monotonic()
            for t in self._lettore.aggiungi(dati):
                partito = self._inviati[t.seq]
                ritardo = adesso - partito if partito and adesso - partito < 1.0 else None
                self._al_pacchetto(t, ritardo)

    def chiudi(self) -> None:
        self._fine.set()
        self._thread.join(timeout=0.5)
        self._seriale.close()
