"""Parte comune ai due nodi: telemetria della scheda -> topic e messaggi di log."""
from __future__ import annotations

import math

from sensor_msgs.msg import JointState

from .protocollo import Telemetria

NOMI_TELEMETRIA = ['theta', 'theta_encoder', 'stima_theta', 'stima_x', 'azione',
                   'eta_ms', 'ritardo_ms']


class MonitorScheda:
    """Pubblica ogni pacchetto su pendolo/stm32 (sensor_msgs/JointState, nomi in
    NOMI_TELEMETRIA: comodo con PlotJuggler o rqt_plot) e scrive nel log i
    cambi di stato e di controllo_errore. Chiamato dal thread seriale."""

    def __init__(self, nodo) -> None:
        self._nodo = nodo
        self._pub = nodo.create_publisher(JointState, 'pendolo/stm32', 10)
        self.ultimo: Telemetria | None = None
        self.pacchetti = 0
        self.ritardi_ms: list[float] = []

    def __call__(self, t: Telemetria, ritardo_s: float | None) -> None:
        self.pacchetti += 1
        ritardo_ms = ritardo_s * 1000.0 if ritardo_s is not None else math.nan
        if ritardo_s is not None:
            self.ritardi_ms.append(ritardo_ms)
        msg = JointState()
        msg.header.stamp = self._nodo.get_clock().now().to_msg()
        msg.name = NOMI_TELEMETRIA
        msg.position = [t.theta, t.theta_encoder, t.stima_theta, t.stima_x, t.azione,
                        float(t.eta_ms), ritardo_ms]
        self._pub.publish(msg)
        self._log_cambi(t)
        self.ultimo = t

    def _log_cambi(self, t: Telemetria) -> None:
        log = self._nodo.get_logger()
        prima = self.ultimo
        if prima is None:
            log.info(f'telemetria dalla scheda: {self.descrivi(t)}')
            return
        if t.errore != prima.errore:
            if t.errore:
                log.warning(f'controllo_errore = {t.errore}: {t.descrizione_errore}')
            else:
                log.info('controllo_errore = 0')
        for nome, ora, allora in (('angolo valido', t.angolo_valido, prima.angolo_valido),
                                  ('theta inizializzato', t.theta_inizializzato, prima.theta_inizializzato),
                                  ('carrello inizializzato', t.carrello_inizializzato, prima.carrello_inizializzato),
                                  ('controllo attivo', t.controllo_attivo, prima.controllo_attivo)):
            if ora != allora:
                (log.info if ora or nome == 'controllo attivo' else log.warning)(
                    f'{nome}: {"si" if ora else "no"}')

    @staticmethod
    def descrivi(t: Telemetria) -> str:
        return (f'angolo valido {"si" if t.angolo_valido else "no"}, '
                f'theta inizializzato {"si" if t.theta_inizializzato else "no"}, '
                f'carrello {"si" if t.carrello_inizializzato else "no"}, '
                f'controllo {"attivo" if t.controllo_attivo else "fermo"}, '
                f'errore {t.errore} ({t.descrizione_errore}), '
                f'theta {math.degrees(t.theta):.1f} deg, encoder {math.degrees(t.theta_encoder):.1f} deg')

    def statistiche_ritardo(self) -> tuple[float, float] | None:
        """Media e massimo (ms) dall'ultima chiamata."""
        if not self.ritardi_ms:
            return None
        r, self.ritardi_ms = self.ritardi_ms, []
        return sum(r) / len(r), max(r)
