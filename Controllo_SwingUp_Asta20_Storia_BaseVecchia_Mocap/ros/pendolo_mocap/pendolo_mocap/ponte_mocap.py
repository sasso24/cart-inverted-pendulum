"""Nodo ROS 2: posa del corpo rigido sull'asta (tracciamento) -> angolo -> Nucleo.

    ros2 run pendolo_mocap ponte_mocap --ros-args --params-file config/ponte_mocap.yaml

A ogni posa ricevuta calcola l'angolo (geometria.py) e lo manda subito alla
scheda: nessun filtro e nessun timer, per non aggiungere ritardo. All'avvio,
se direzione_corpo non e' data, si calibra con il pendolo fermo appeso.
"""
from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from .comune import MonitorScheda
from .geometria import AngoloPendolo, normalizza_quaternione
from .seriale import Collegamento


class PonteMocap(Node):
    def __init__(self) -> None:
        super().__init__('ponte_mocap')
        par = self.declare_parameter
        porta = par('porta', '/dev/ttyACM0').value
        baud = par('baud', 115200).value
        topic = par('topic_posa', '/vrpn_mocap/pendolo/pose').value
        asse_su = par('asse_su', [0.0, 0.0, 1.0]).value
        asse_carrello = par('asse_carrello', [1.0, 0.0, 0.0]).value
        # Tutti zero = calibrazione all'avvio con il pendolo fermo appeso.
        direzione = par('direzione_corpo', [0.0, 0.0, 0.0]).value
        self._n_calibrazione = par('campioni_calibrazione', 200).value
        self._dispersione_max = math.radians(par('dispersione_max_gradi', 1.0).value)
        # Pose identiche di fila: il driver ripete l'ultima posa a corpo perso.
        self._ripetute_max = par('pose_ripetute_max', 10).value
        self._fuori_piano_max = math.radians(par('fuori_piano_max_gradi', 15.0).value)

        self._angolo = AngoloPendolo(asse_su, asse_carrello,
                                     direzione if any(direzione) else None)
        self._campioni: list[tuple[float, float, float, float]] = []
        self._ultimo_q = None
        self._ripetute = 0
        self._pose = 0
        self._ultima_posa = time.monotonic()

        self._monitor = MonitorScheda(self)
        self._link = Collegamento(porta, baud, self._monitor)
        self._pub_angolo = self.create_publisher(Float64, 'pendolo/angolo', 10)
        self.create_subscription(PoseStamped, topic, self._posa, qos_profile_sensor_data)
        self.create_service(Trigger, '~/calibra', self._calibra)
        self.create_timer(5.0, self._rapporto)
        log = self.get_logger()
        log.info(f'{topic} -> {porta} a {baud} baud')
        if self._angolo.calibrato:
            log.info(f'direzione dell\'asta nel corpo rigido: {self._angolo.direzione_corpo}')
        else:
            log.info('calibrazione: tenere il pendolo FERMO e APPESO')

    def _posa(self, msg: PoseStamped) -> None:
        self._pose += 1
        self._ultima_posa = time.monotonic()
        o = msg.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        self._ripetute = self._ripetute + 1 if q == self._ultimo_q else 0
        self._ultimo_q = q
        try:
            q = normalizza_quaternione(q)
        except ValueError:
            q = None
        if q is None or (self._ripetute_max and self._ripetute >= self._ripetute_max):
            # Corpo rigido perso: la scheda lo sa subito e scade dopo 50 ms.
            if self._angolo.calibrato:
                self._link.invia_angolo(0.0, tracciato=False)
            return
        if not self._angolo.calibrato:
            self._accumula(q)
            return
        theta = self._angolo.angolo(q)
        self._link.invia_angolo(theta)
        self._pub_angolo.publish(Float64(data=theta))
        fuori = self._angolo.fuori_piano(q)
        if abs(fuori) > self._fuori_piano_max:
            self.get_logger().warning(
                f'asta {math.degrees(fuori):.0f} deg fuori dal piano di oscillazione: '
                'asse_carrello o asse_su sbagliati?', throttle_duration_sec=2.0)

    def _accumula(self, q) -> None:
        self._campioni.append(q)
        if len(self._campioni) < self._n_calibrazione:
            return
        campioni, self._campioni = self._campioni, []
        d = self._angolo.calibra(campioni)
        dispersione = max(self._angolo.scostamento_da_appeso(c) for c in campioni)
        log = self.get_logger()
        if dispersione > self._dispersione_max:
            self._angolo.direzione_corpo = None
            log.warning(f'pendolo non fermo durante la calibrazione ({math.degrees(dispersione):.2f} deg): riprovo')
            return
        log.info(f'calibrato. Per saltare la calibrazione: direzione_corpo: '
                 f'[{d[0]:.6f}, {d[1]:.6f}, {d[2]:.6f}]')

    def _calibra(self, richiesta, risposta):
        self._angolo.direzione_corpo = None
        self._campioni = []
        risposta.success = True
        risposta.message = ('calibrazione in corso: pendolo fermo appeso. Nel frattempo nessun '
                            'angolo alla scheda (in controllo si ferma con errore 12).')
        self.get_logger().info(risposta.message)
        return risposta

    def _rapporto(self) -> None:
        log = self.get_logger()
        if time.monotonic() - self._ultima_posa > 0.5:
            log.warning('nessuna posa dal tracciamento (topic_posa giusto? corpo rigido visibile?)')
        t = self._monitor.ultimo
        if t is None:
            log.warning('nessuna telemetria dalla scheda (porta giusta? firmware _Mocap caricato?)')
            return
        r = self._monitor.statistiche_ritardo()
        log.info(f'pose {self._pose / 5.0:.0f}/s, {MonitorScheda.descrivi(t)}, eta {t.eta_ms} ms'
                  + (f', ritardo {r[0]:.1f} ms (max {r[1]:.1f})' if r else ''))
        self._pose = 0

    def destroy_node(self) -> None:
        self._link.chiudi()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = PonteMocap()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
