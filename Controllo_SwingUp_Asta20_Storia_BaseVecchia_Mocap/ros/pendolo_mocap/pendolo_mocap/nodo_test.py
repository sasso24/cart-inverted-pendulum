"""Nodo ROS 2 di prova: angoli sintetici alla Nucleo, senza tracciamento.

    ros2 run pendolo_mocap nodo_test --ros-args -p modo:=appeso -p durata_s:=30.0

Verifica il collegamento con il firmware _Mocap: campioni persi, pacchetti
scartati, eta' e ritardo dei campioni, angolo usato dalla scheda = angolo
mandato + offset costante, reazione ai guasti. Modi (parametro modo):
  appeso        pi + rumore: la scheda prende il riferimento (theta inizializzato)
  oscillazione  pi + ampiezza*exp(-t/smorzamento)*sin(2 pi t/periodo): la scheda
                aspetta che si fermi, poi prende il riferimento
  rotazione     giri continui (passaggi per +/-pi): l'angolo deve restare coerente
  alto          0 + rumore (partenza dall'alto)
  topic         angolo (rad) da pendolo/angolo_test, std_msgs/Float64
Guasti (parametro guasto, da inizio_guasto_s per durata_guasto_s):
  pausa         nessun pacchetto: dopo 50 ms angolo non valido (errore 12 in controllo)
  non_tracciato pacchetti con corpo rigido perso: come pausa
  salto         +1 rad: errore 9 in controllo
  checksum      checksum sbagliato: pacchetti scartati, poi come pausa
  rumore        byte casuali fra i pacchetti: la scheda deve risincronizzarsi
                senza mai perdere l'angolo (byte fuori pacchetto: ignorati, non contati)

ATTENZIONE: con il firmware normale (PROVA_COLLEGAMENTO 0) dopo il riferimento
parte l'homing e il carrello si muove; se si preme B1 la rete muove il carrello
in base all'angolo FINTO. Per la prova da tavolo usare PROVA_COLLEGAMENTO 1.
"""
from __future__ import annotations

import math
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from .comune import MonitorScheda
from .protocollo import Telemetria, avvolgi, pacchetto_angolo
from .seriale import Collegamento

MODI = ('appeso', 'oscillazione', 'rotazione', 'alto', 'topic')
GUASTI = ('nessuno', 'pausa', 'non_tracciato', 'salto', 'checksum', 'rumore')


class NodoTest(Node):
    def __init__(self) -> None:
        super().__init__('nodo_test_pendolo')
        par = self.declare_parameter
        porta = par('porta', '/dev/ttyACM0').value
        baud = par('baud', 115200).value
        self._modo = par('modo', 'appeso').value
        self._frequenza = par('frequenza_hz', 200.0).value
        self._rumore = math.radians(par('rumore_gradi', 0.05).value)
        self._ampiezza = math.radians(par('ampiezza_gradi', 20.0).value)
        self._periodo = par('periodo_s', 1.5).value
        self._smorzamento = par('smorzamento_s', 3.0).value
        self._giri_s = par('giri_s', 0.5).value
        self._guasto = par('guasto', 'nessuno').value
        self._inizio_guasto = par('inizio_guasto_s', 10.0).value
        self._durata_guasto = par('durata_guasto_s', 0.3).value
        self._durata = par('durata_s', 0.0).value   # 0 = fino a Ctrl-C
        if self._modo not in MODI:
            raise ValueError(f'modo {self._modo!r}: uno di {MODI}')
        if self._guasto not in GUASTI:
            raise ValueError(f'guasto {self._guasto!r}: uno di {GUASTI}')

        self._rng = random.Random(1)
        self._angolo_topic = math.pi
        self._inviati: list[float | None] = [None] * 65536   # angolo per seq
        self._differenze: list[float] = []
        self._eta_max = 0
        self._validita: list[tuple[float, bool]] = []   # (t, angolo valido) per i guasti
        self._prima: Telemetria | None = None
        self._inizializzato_a: float | None = None
        self._finito = False

        self._monitor = MonitorScheda(self)
        self._link = Collegamento(porta, baud, self._telemetria)
        if self._modo == 'topic':
            self.create_subscription(Float64, 'pendolo/angolo_test', self._da_topic, 10)
        self._t0 = time.monotonic()
        self.create_timer(1.0 / self._frequenza, self._invia)
        self.create_timer(2.0, self._rapporto)
        self.get_logger().info(f'modo {self._modo}, guasto {self._guasto}, {self._frequenza:.0f} Hz '
                               f'su {porta}')

    # --- angoli mandati -------------------------------------------------
    def _da_topic(self, msg: Float64) -> None:
        self._angolo_topic = msg.data

    def _angolo(self, t: float) -> float:
        if self._modo == 'appeso':
            theta = math.pi
        elif self._modo == 'oscillazione':
            decadimento = math.exp(-t / self._smorzamento) if self._smorzamento > 0 else 1.0
            theta = math.pi + self._ampiezza * decadimento * math.sin(2*math.pi*t / self._periodo)
        elif self._modo == 'rotazione':
            theta = math.pi + 2*math.pi*self._giri_s*t
        elif self._modo == 'alto':
            theta = 0.0
        else:
            theta = self._angolo_topic
        return avvolgi(theta + self._rng.gauss(0.0, self._rumore))

    def _in_guasto(self, t: float) -> bool:
        return (self._guasto != 'nessuno'
                and self._inizio_guasto <= t < self._inizio_guasto + self._durata_guasto)

    def _invia(self) -> None:
        if self._finito:
            return
        t = time.monotonic() - self._t0
        if self._durata > 0 and t >= self._durata:
            self._finito = True
            self._resoconto()
            raise SystemExit
        theta = self._angolo(t)
        guasto = self._in_guasto(t)
        if guasto and self._guasto == 'pausa':
            return
        if guasto and self._guasto == 'salto':
            theta = avvolgi(theta + 1.0)
        if guasto and self._guasto == 'rumore':
            self._link.invia_byte(bytes(self._rng.randrange(256) for _ in range(self._rng.randint(1, 8))))
        if guasto and self._guasto == 'checksum':
            p = bytearray(pacchetto_angolo(theta, 0))
            p[-1] ^= 0xFF
            self._link.invia_byte(bytes(p))
            return
        tracciato = not (guasto and self._guasto == 'non_tracciato')
        seq = self._link.invia_angolo(theta, tracciato)
        self._inviati[seq] = theta if tracciato else None

    # --- telemetria -----------------------------------------------------
    def _telemetria(self, tel: Telemetria, ritardo_s: float | None) -> None:
        self._monitor(tel, ritardo_s)
        adesso = time.monotonic() - self._t0
        self._validita.append((adesso, tel.angolo_valido))
        if tel.theta_inizializzato and self._inizializzato_a is None:
            self._inizializzato_a = adesso
            self.get_logger().info(f'riferimento dell\'angolo preso dopo {adesso:.1f} s')
        if self._prima is None:
            self._prima = tel
        if not tel.angolo_valido:
            return
        self._eta_max = max(self._eta_max, tel.eta_ms)
        mandato = self._inviati[tel.seq] if ritardo_s is not None else None
        if mandato is not None and tel.theta_inizializzato:
            # Angolo della scheda = angolo mandato con quel seq + offset costante.
            self._differenze.append(avvolgi(tel.theta - mandato))

    def _offset(self) -> tuple[float, float] | None:
        """Offset medio e massimo scarto da esso (rad)."""
        d = self._differenze
        if not d:
            return None
        media = math.atan2(sum(map(math.sin, d)), sum(map(math.cos, d)))
        return media, max(abs(avvolgi(x - media)) for x in d)

    def _rapporto(self) -> None:
        t = self._monitor.ultimo
        log = self.get_logger()
        if t is None:
            log.warning('nessuna telemetria dalla scheda (porta? firmware _Mocap? scheda in Error_Handler?)')
            return
        r = self._monitor.statistiche_ritardo()
        o = self._offset()
        prima = self._prima or t
        ricevuti = (t.ricevuti - prima.ricevuti) % 65536
        scartati = (t.scartati - prima.scartati) % 65536
        log.info(f'mandati {self._link.inviati}, arrivati alla scheda {ricevuti}, scartati {scartati}, '
                 f'eta {t.eta_ms} ms (max {self._eta_max})'
                 + (f', ritardo {r[0]:.1f} ms (max {r[1]:.1f})' if r else '')
                 + (f', offset {math.degrees(o[0]):.2f} deg (scarto max {o[1]:.1e} rad)' if o else '')
                 + f' | {MonitorScheda.descrivi(t)}')

    # --- esito finale ---------------------------------------------------
    def _resoconto(self) -> None:
        log = self.get_logger()
        ultimo, prima = self._monitor.ultimo, self._prima
        esiti: list[tuple[str, bool]] = []
        esiti.append(('telemetria ricevuta', ultimo is not None))
        if ultimo is not None and prima is not None:
            mandati = max(1, self._link.inviati)
            ricevuti = (ultimo.ricevuti - prima.ricevuti) % 65536
            # Senza guasti quasi tutti i campioni arrivano (la prima telemetria parte dopo i primi).
            if self._guasto in ('nessuno', 'salto'):
                esiti.append((f'campioni arrivati {ricevuti}/{mandati}', ricevuti >= 0.98*mandati - 20))
            if self._guasto in ('nessuno', 'salto', 'rumore'):
                esiti.append(('angolo sempre valido dopo il primo campione',
                              all(v for t, v in self._validita if t > 0.5)))
            if self._guasto in ('pausa', 'non_tracciato', 'checksum') and self._durata_guasto >= 0.15:
                durante = [v for t, v in self._validita
                           if self._inizio_guasto + 0.1 <= t < self._inizio_guasto + self._durata_guasto]
                dopo = [v for t, v in self._validita if t >= self._inizio_guasto + self._durata_guasto + 0.1]
                esiti.append(('angolo non valido durante il guasto', bool(durante) and not any(durante)))
                esiti.append(('angolo di nuovo valido dopo il guasto', bool(dopo) and all(dopo)))
            if self._guasto == 'checksum':
                esiti.append(('pacchetti errati contati dalla scheda',
                              (ultimo.scartati - prima.scartati) % 65536 > 0))
            if self._modo in ('appeso', 'oscillazione'):
                esiti.append(('riferimento dell\'angolo preso', self._inizializzato_a is not None))
            o = self._offset()
            if o is not None:
                esiti.append((f'angolo della scheda = mandato + {math.degrees(o[0]):.2f} deg '
                              f'(scarto {o[1]:.1e} rad)', o[1] < 1e-4))
            esiti.append((f'eta massima del campione {self._eta_max} ms', self._eta_max <= 50))
        for nome, ok in esiti:
            (log.info if ok else log.error)(f'{"OK  " if ok else "NO  "} {nome}')
        log.info('PROVA RIUSCITA' if all(ok for _, ok in esiti) else 'PROVA FALLITA')

    def destroy_node(self) -> None:
        self._link.chiudi()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = NodoTest()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not nodo._finito:
            nodo._resoconto()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
