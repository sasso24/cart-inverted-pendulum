"""Protocollo e geometria, senza ROS: python3 -m pytest test/ (o python3 test/test_senza_ros.py)."""
import math
from pathlib import Path
import random
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pendolo_mocap import geometria, protocollo  # noqa: E402


def quaternione_asse_angolo(asse, angolo):
    a = geometria.normalizza(asse)
    s = math.sin(angolo / 2)
    return (a[0]*s, a[1]*s, a[2]*s, math.cos(angolo / 2))


def prodotto(q, r):
    x1, y1, z1, w1 = q
    x2, y2, z2, w2 = r
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2)


def test_pacchetto_angolo():
    p = protocollo.pacchetto_angolo(-3.0, 0x1234)
    assert len(p) == protocollo.LUNGHEZZA_ANGOLO == 11
    assert p[:2] == b'\xa5\x5a' and p[2] == 0x4D and p[3] == 1 and p[4:6] == b'\x34\x12'
    assert sum(p[2:]) % 256 == 0
    assert protocollo.pacchetto_angolo(0.0, 70000, tracciato=False)[3:6] == b'\x00\x70\x11'


def test_lettore_telemetria_a_pezzi_e_con_rumore():
    rng = random.Random(0)
    attese = [protocollo.Telemetria(stato=i % 16, errore=i % 13, eta_ms=i, seq=1000 + i, ricevuti=i,
                                    scartati=2*i, tick_ms=20*i, theta=0.25*i, theta_encoder=-0.5,
                                    stima_theta=1.0, stima_x=0.125, azione=-1.0) for i in range(40)]
    flusso = bytearray()
    for t in attese:
        flusso += bytes(rng.randrange(256) for _ in range(rng.randint(0, 5))) + b'\xa5'
        flusso += protocollo.pacchetto_telemetria(t)
    guasto = bytearray(protocollo.pacchetto_telemetria(attese[0]))
    guasto[-1] ^= 1
    flusso += guasto
    lettore, letti, i = protocollo.LettoreTelemetria(), [], 0
    while i < len(flusso):
        n = rng.randint(1, 50)
        letti += lettore.aggiungi(bytes(flusso[i:i + n]))
        i += n
    assert letti == attese
    assert lettore.scartati >= 1


def test_avvolgi():
    assert protocollo.avvolgi(math.pi) == math.pi
    assert protocollo.avvolgi(-math.pi) == math.pi
    assert abs(protocollo.avvolgi(3*math.pi/2) + math.pi/2) < 1e-12


def test_angolo_convenzione_del_firmware():
    # z in alto, binario lungo x, cerniera su y come in cart_pendolo_asta20.xml.
    # Corpo rigido montato storto sull'asta: la calibrazione da appeso lo assorbe.
    montaggio = prodotto(quaternione_asse_angolo((1, 2, 3), 0.7), (0, 0, 0, 1))
    def posa(theta):  # rotazione dell'asta di theta attorno a +y, a partire da "su"
        return prodotto(quaternione_asse_angolo((0, 1, 0), theta), montaggio)
    a = geometria.AngoloPendolo((0, 0, 1), (1, 0, 0))
    a.calibra([prodotto(quaternione_asse_angolo((0, 1, 0), math.pi), montaggio)] * 5)
    for theta in (0.0, 0.3, -0.3, 1.5, -2.9, math.pi):
        assert abs(protocollo.avvolgi(a.angolo(posa(theta)) - theta)) < 1e-9
    # theta > 0: punta del braccio lungo verso +x.
    assert geometria.ruota(posa(0.3), a.direzione_corpo)[0] > 0
    assert abs(a.fuori_piano(posa(1.0))) < 1e-9
    assert abs(a.scostamento_da_appeso(posa(math.pi - 0.1)) - 0.1) < 1e-9


def test_angolo_y_in_alto_e_binario_obliquo():
    # Motive in Y-up, binario a 30 gradi nel piano orizzontale x-z.
    su = (0, 1, 0)
    binario = (math.cos(0.5), 0.0, math.sin(0.5))
    cerniera = geometria._vettoriale(su, binario)   # normale al piano di oscillazione
    a = geometria.AngoloPendolo(su, (binario[0], 0.2, binario[2]))   # componente verticale ignorata
    montaggio = quaternione_asse_angolo((0.3, -1, 0.2), 2.0)
    # Rotazione di theta attorno a su x binario: la punta va dalla verticale verso +binario.
    def posa(theta):
        return prodotto(quaternione_asse_angolo(cerniera, theta), montaggio)
    a.calibra([posa(math.pi)])
    for theta in (0.2, -1.0, 2.5):
        assert abs(protocollo.avvolgi(a.angolo(posa(theta)) - theta)) < 1e-9


if __name__ == '__main__':   # anche senza pytest
    for nome, f in list(globals().items()):
        if nome.startswith('test_'):
            f()
            print('OK', nome)
