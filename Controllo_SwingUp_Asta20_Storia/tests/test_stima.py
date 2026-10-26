"""Confronta lo stimatore C del firmware (Core/Src/stima.c) con stima.py.

python3 Controllo_SwingUp_Asta20_Storia/tests/test_stima.py   (dalla radice del repository)
Serve l'ambiente Python del progetto cart_pendolo_velocita_rl (numpy, mujoco).
"""
import ctypes
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PY = ROOT.parent / 'cart_pendolo_velocita_rl'
sys.path.insert(0, str(PY))
from stima import ParametriStimatore, StimatoreStato  # noqa: E402


class Stima(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in
                ('x', 'theta', 'omega', 'velocita', 'velocita_precedente', 'offset')]


def compila(cartella, header=None):
    """Libreria dallo stima.c del firmware; header = config_controllo.h alternativo."""
    lib = Path(cartella) / 'libstima.so'
    inc = []
    if header is not None:
        (Path(cartella) / 'config_controllo.h').write_text(header)
        # stima.h include "config_controllo.h" dalla propria cartella: copiarlo accanto.
        (Path(cartella) / 'stima.h').write_text((ROOT / 'Core/Inc/stima.h').read_text())
        inc = ['-I', str(cartella)]
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-O2', '-shared', '-fPIC', *inc, '-I', str(ROOT / 'Core/Inc'),
                    str(ROOT / 'Core/Src/stima.c'), '-lm', '-o', str(lib)], check=True)
    c = ctypes.CDLL(str(lib))
    c.Stima_Reset.argtypes = [ctypes.POINTER(Stima)] + [ctypes.c_float]*3
    c.Stima_Predici.argtypes = [ctypes.POINTER(Stima), ctypes.c_float, ctypes.c_float]
    c.Stima_Correggi.argtypes = [ctypes.POINTER(Stima), ctypes.c_float, ctypes.c_float]
    c.Stima_Wrap.argtypes = [ctypes.c_float]
    c.Stima_Wrap.restype = ctypes.c_float
    return c


def costanti_header(header=None):
    """Le #define STIMA_* di config_controllo.h (scritte da esporta_firmware.py)."""
    header = header or (ROOT / 'Core/Inc/config_controllo.h').read_text()
    valori = {}
    for nome in ('k_g', 'k_a', 'k_d', 'l_theta', 'l_omega', 'l_x', 'l_b', 'b_max_rad'):
        trovato = re.search(rf'#define STIMA_{nome.upper()}\s+\(?(-?[0-9.eE+-]+)f', header)
        if trovato:            # l_b e b_max_rad mancano negli header prima del 16/10
            valori[nome] = float(trovato[1])
    return ParametriStimatore(**valori)


def header_con_offset():
    """config_controllo.h attuale con l'offset attivo (guadagni a 20 ms)."""
    import stima
    header = (ROOT / 'Core/Inc/config_controllo.h').read_text()
    p = costanti_header(header)
    g = stima.guadagni_con_offset(p, 0.02)
    righe = [r for r in header.splitlines()
             if not re.match(r'#define STIMA_(L_THETA|L_OMEGA|L_B|B_MAX_RAD)\b', r)]
    extra = [f'#define STIMA_L_THETA  {g["l_theta"]!r}f', f'#define STIMA_L_OMEGA  {g["l_omega"]!r}f',
             f'#define STIMA_L_B      ({g["l_b"]!r}f)', '#define STIMA_B_MAX_RAD 0.05f']
    fine = righe.index('#endif')
    return '\n'.join(righe[:fine] + extra + righe[fine:]) + '\n'


def confronta(c, seed, header=None):
    rng = np.random.default_rng(seed)
    py, cs = StimatoreStato(costanti_header(header)), Stima()
    x0, th0 = rng.uniform(-0.2, 0.2), rng.uniform(-math.pi, math.pi)
    py.reset(x0, th0)
    c.Stima_Reset(ctypes.byref(cs), x0, th0, 0.)
    v, theta_vero, errore = 0., th0, 0.
    for k in range(3000):          # 60 s a 50 Hz
        for _ in range(10):        # 2 ms
            v = float(np.clip(v + rng.normal(0, 0.02), -0.48, 0.48))
            py.predici(v, 0.002)
            c.Stima_Predici(ctypes.byref(cs), v, 0.002)
        theta_vero += rng.normal(0, 0.05)        # misure che attraversano +/-pi
        x_m = rng.normal(0, 0.01)
        th_m = math.remainder(theta_vero + 0.04 + rng.normal(0, 0.017), 2*math.pi)
        py.correggi(x_m, th_m)
        c.Stima_Correggi(ctypes.byref(cs), x_m, th_m)
        d_theta = abs(math.remainder(py.theta - cs.theta, 2*math.pi))
        errore = max(errore, d_theta, abs(py.omega - cs.omega)/10, abs(py.x - cs.x),
                     abs(py.velocita - cs.velocita), abs(py.offset - cs.offset))
        assert -math.pi < cs.theta <= math.pi
    return errore


def main():
    with tempfile.TemporaryDirectory(prefix='stima-c-') as cartella:
        c = compila(cartella)
        for a in (math.pi, -math.pi, 3*math.pi, 0.1, -7.):
            w = c.Stima_Wrap(a)
            assert -math.pi < w <= math.pi + 1e-6 and abs(math.remainder(w - a, 2*math.pi)) < 1e-5
        errori = [confronta(c, s) for s in range(20)]
    with tempfile.TemporaryDirectory(prefix='stima-c-offset-') as cartella:
        header = header_con_offset()
        c = compila(cartella, header)
        errori_offset = [confronta(c, s, header) for s in range(20)]
    # float32 (C) contro float64 (Python): differenza massima su 20 x 60 s.
    assert max(errori) < 2e-3, errori
    assert max(errori_offset) < 2e-3, errori_offset
    print(f'OK: stimatore C = stima.py (differenza massima {max(errori):.2e}; con offset '
          f'{max(errori_offset):.2e}; 2 x 20 sequenze da 60 s)')


if __name__ == '__main__':
    main()
