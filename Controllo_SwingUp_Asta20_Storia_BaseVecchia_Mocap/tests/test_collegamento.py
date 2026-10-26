"""Prova su host del collegamento nodo ROS <-> firmware, senza scheda e senza ROS.

Il codice reale di main.c (ricezione dell'angolo, riferimento, quiete, timeout,
telemetria) gira sul PC in tempo reale dietro uno pseudo-terminale, come in
PROVA_COLLEGAMENTO (niente homing, motore mai comandato). Dall'altra parte
girano i nodi veri di ros/pendolo_mocap (nodo_test e ponte_mocap) con un rclpy
finto minimo: verifica seriale.py, protocollo.py, la geometria e la logica dei
nodi, non il middleware ROS.

python3 Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap/tests/test_collegamento.py
(ambiente con pyserial, per esempio cart_pendolo_stepper_rl/.venv). Solo Linux/macOS.
"""
import math
import os
from pathlib import Path
import pty
import random
import re
import subprocess
import sys
import tempfile
import time
import tty
import types

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros/pendolo_mocap'))

sorgente_test = (ROOT / 'tests/test_controllo.py').read_text()
MOCK = re.search(r"MOCK = r'''(.*?)'''", sorgente_test, re.S)[1]
SCHEDA = r'''
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
static int uart_fd = -1;
static uint64_t t0_ms;
static uint64_t adesso_ms(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec*1000U + (uint64_t)t.tv_nsec/1000000U;
}
/* Un millisecondo della scheda: tempo reale, byte ricevuti all'ISR, SysTick, byte trasmessi. */
static void millisecondo(void) {
    usleep(500);
    uint32_t t = (uint32_t)(adesso_ms() - t0_ms);
    if (t > tick) tick = t;
    uint8_t buf[256];
    struct pollfd p = {uart_fd, POLLIN, 0};
    while (poll(&p, 1, 0) > 0) {
        ssize_t n = read(uart_fd, buf, sizeof buf);
        if (n <= 0) exit(0);   /* PC disconnesso: fine della prova */
        for (ssize_t i = 0; i < n; i++) { usart2.SR = USART_SR_RXNE; usart2.DR = buf[i]; Mocap_UartIrq(); }
    }
    usart2.SR = 0;
    Controllo_TelemetriaTick();
    uint8_t out[64]; size_t k = 0;
    while ((usart2.CR1 & USART_CR1_TXEIE) && k < sizeof out) {
        usart2.SR = USART_SR_TXE; usart2.DR = 0x100; Mocap_UartIrq();
        if (usart2.DR != 0x100) out[k++] = (uint8_t)usart2.DR;
    }
    usart2.SR = 0;
    if (k && write(uart_fd, out, k) != (ssize_t)k) exit(1);
}
int main(int argc, char **argv) {
    uart_fd = atoi(argv[1]);
    t0_ms = adesso_ms();
    htim2.Instance=&timer2; htim3.Instance=&timer3; huart2.Instance=&usart2;
    htim3.Channel=HAL_TIM_ACTIVE_CHANNEL_1;
    reti_candidate_stima[0] = (GuadagniStima){ STIMA_L_THETA, STIMA_L_OMEGA, STIMA_L_B, STIMA_L_X };
    output_value = 0.0f;
    assert(Rete_Init());
    delay_hook = millisecondo;
    Mocap_Avvia();
    /* Come main() con PROVA_COLLEGAMENTO 1. */
    if (Inizializza_Theta() != HAL_OK) { fprintf(stderr, "riferimento non preso\n"); return 2; }
    Leggi_encoder();
    ultimo_theta_valido = Leggi_theta();
    ultimo_theta_rad = theta_rad;
    ultimo_campione_ms = HAL_GetTick();
    pulsante_inizializzato = 1;
    while (1) {
        uint32_t periodi = rete_periodi;
        Rete_Process();
        if (rete_periodi != periodi) Telemetria_Invia();
        HAL_Delay(1);
    }
}
'''


# --- rclpy finto: solo quello che usano i due nodi --------------------------
class _Log:
    def __init__(self, righe):
        self.righe = righe

    def _scrivi(self, livello, msg, **_):
        self.righe.append((livello, msg))
        print(f'  [{livello}] {msg}')

    def debug(self, msg, **k): self._scrivi('debug', msg, **k)
    def info(self, msg, **k): self._scrivi('info', msg, **k)
    def warning(self, msg, **k): self._scrivi('warn', msg, **k)
    def error(self, msg, **k): self._scrivi('errore', msg, **k)


class _Nodo:
    parametri = {}
    sottoscrizioni = {}

    def __init__(self, nome):
        self.righe = []
        self._log = _Log(self.righe)
        self.timer = []

    def declare_parameter(self, nome, predefinito):
        return types.SimpleNamespace(value=self.parametri.get(nome, predefinito))

    def create_publisher(self, tipo, topic, qos):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_subscription(self, tipo, topic, funzione, qos):
        self.sottoscrizioni[topic] = funzione

    def create_service(self, tipo, nome, funzione):
        pass

    def create_timer(self, periodo, funzione):
        self.timer.append([periodo, funzione, time.monotonic() + periodo])

    def get_logger(self):
        return self._log

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))

    def destroy_node(self):
        pass


def _gira(nodo):
    while True:
        adesso = time.monotonic()
        for t in nodo.timer:
            if adesso >= t[2]:
                t[2] += t[0]
                t[1]()
        time.sleep(0.0003)


class _Messaggio:
    def __init__(self, **campi):
        self.header = types.SimpleNamespace(stamp=None)
        self.__dict__.update(campi)


def installa_rclpy_finto():
    moduli = {
        'rclpy': types.SimpleNamespace(init=lambda args=None: None, ok=lambda: False,
                                       shutdown=lambda: None, spin=_gira),
        'rclpy.node': types.SimpleNamespace(Node=_Nodo),
        'rclpy.qos': types.SimpleNamespace(qos_profile_sensor_data=None),
        'std_msgs': types.SimpleNamespace(), 'std_msgs.msg': types.SimpleNamespace(Float64=_Messaggio),
        'sensor_msgs': types.SimpleNamespace(), 'sensor_msgs.msg': types.SimpleNamespace(JointState=_Messaggio),
        'geometry_msgs': types.SimpleNamespace(),
        'geometry_msgs.msg': types.SimpleNamespace(PoseStamped=_Messaggio),
        'std_srvs': types.SimpleNamespace(), 'std_srvs.srv': types.SimpleNamespace(Trigger=object),
    }
    sys.modules.update(moduli)


def prova_nodo_test(porta, **parametri):
    from pendolo_mocap import nodo_test
    _Nodo.parametri = dict(porta=porta, **parametri)
    print(f'nodo_test {parametri}')
    nodo = nodo_test.NodoTest()
    try:
        _gira(nodo)
    except SystemExit:
        pass
    nodo.destroy_node()
    assert ('info', 'PROVA RIUSCITA') in nodo.righe, parametri
    return nodo


def prova_ponte(porta):
    """Pose sintetiche a 200 Hz: appeso fermo (calibrazione e riferimento della
    scheda), poi oscillazione. Il corpo rigido e' montato storto e il
    tracciamento e' in Y-up: angolo della scheda = angolo vero (offset ~0)."""
    from pendolo_mocap import geometria, ponte_mocap

    def asse_angolo(asse, a):
        u = geometria.normalizza(asse)
        return (u[0]*math.sin(a/2), u[1]*math.sin(a/2), u[2]*math.sin(a/2), math.cos(a/2))

    def prodotto(q, r):
        x1, y1, z1, w1 = q
        x2, y2, z2, w2 = r
        return (w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2)
    su, binario = (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)
    cerniera = geometria._vettoriale(su, binario)
    montaggio = asse_angolo((0.2, 1.0, -0.4), 1.1)
    vero = []
    rumore = random.Random(2)

    def theta_vero(t):
        return math.pi if t < 2.5 else math.pi - 0.4*math.sin(2*math.pi*(t - 2.5))

    _Nodo.parametri = dict(porta=porta, topic_posa='/posa', asse_su=list(su), asse_carrello=list(binario))
    print('ponte_mocap')
    nodo = ponte_mocap.PonteMocap()
    t0 = time.monotonic()

    def posa():
        t = time.monotonic() - t0
        if t >= 4.0:
            raise SystemExit
        # Rumore del tracciamento (0,01 gradi): pose mai identiche al bit, altrimenti
        # il ponte le tratta come corpo rigido perso (pose_ripetute_max).
        theta = theta_vero(t) + rumore.gauss(0.0, 1.7e-4)
        vero.append((time.monotonic(), theta))
        q = prodotto(asse_angolo(cerniera, theta), montaggio)
        o = types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])
        _Nodo.sottoscrizioni['/posa'](types.SimpleNamespace(pose=types.SimpleNamespace(orientation=o)))
    nodo.create_timer(0.005, posa)
    try:
        _gira(nodo)
    except SystemExit:
        pass
    ultimo = nodo._monitor.ultimo
    nodo.destroy_node()
    assert any('calibrato' in m for _, m in nodo.righe)
    assert ultimo is not None and ultimo.theta_inizializzato and ultimo.angolo_valido
    # Angolo della scheda = angolo vero di poco prima (ritardo del collegamento).
    vicino = min(abs(math.remainder(ultimo.theta - th, 2*math.pi)) for _, th in vero[-10:])
    assert vicino < 0.02, (ultimo, vero[-10:])
    print(f'OK: ponte_mocap, angolo della scheda {math.degrees(ultimo.theta):.1f} deg')


def main():
    installa_rclpy_finto()
    stima = ((ROOT / 'Core/Inc/config_controllo.h').read_text()
             + (ROOT / 'Core/Inc/stima.h').read_text().replace('#include "config_controllo.h"', '')
             + (ROOT / 'Core/Src/stima.c').read_text().replace('#include "stima.h"', ''))
    source = (ROOT / 'Core/Src/main.c').read_text()

    def section(name):
        return source.split(f'/* USER CODE BEGIN {name} */', 1)[1].split(f'/* USER CODE END {name} */', 1)[0]
    unit = (MOCK + stima + section('PD')
            + '\nTIM_HandleTypeDef htim2, htim3;\nUART_HandleTypeDef huart2;\n')
    for name in ('PV', 'PFP', '0', '4'):
        unit += section(name)
    unit += SCHEDA
    with tempfile.TemporaryDirectory(prefix='collegamento-test-') as tmp:
        tmp = Path(tmp)
        (tmp / 'scheda.c').write_text(unit)
        subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-O2', '-Wall', '-Wno-unused-function',
                        '-Wno-unused-parameter', str(tmp / 'scheda.c'), '-lm', '-o', str(tmp / 'scheda')],
                       check=True)
        for prova in ('nodo_test', 'ponte'):
            master, slave = pty.openpty()
            tty.setraw(slave)
            tty.setraw(master)
            scheda = subprocess.Popen([str(tmp / 'scheda'), str(master)], pass_fds=[master])
            try:
                porta = os.ttyname(slave)
                if prova == 'nodo_test':
                    # Riferimento, poi angolo assente per 0,3 s e di nuovo presente.
                    prova_nodo_test(porta, modo='appeso', guasto='pausa', inizio_guasto_s=2.5,
                                    durata_guasto_s=0.3, durata_s=4.0)
                    prova_nodo_test(porta, modo='rotazione', giri_s=1.0, guasto='nessuno', durata_s=2.5)
                    prova_nodo_test(porta, modo='appeso', guasto='non_tracciato', inizio_guasto_s=1.0,
                                    durata_guasto_s=0.3, durata_s=2.0)
                    prova_nodo_test(porta, modo='oscillazione', guasto='rumore', inizio_guasto_s=0.5,
                                    durata_guasto_s=1.0, durata_s=2.5)
                    prova_nodo_test(porta, modo='appeso', guasto='checksum', inizio_guasto_s=0.5,
                                    durata_guasto_s=0.3, durata_s=1.5)
                    print('OK: nodo_test (pausa, rotazione oltre +/-180, corpo perso, rumore, checksum)')
                else:
                    prova_ponte(porta)
            finally:
                scheda.kill()
                scheda.wait()
                os.close(master)
                os.close(slave)


if __name__ == '__main__':
    main()
