"""Confronta lo stimatore C del firmware (Core/Src/stima.c) con stima.py.

python3 Prova_controllo/tests/test_stima.py   (dalla radice del repository)
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
                ('x', 'theta', 'omega', 'velocita', 'velocita_precedente')]


def compila(cartella):
    lib = Path(cartella) / 'libstima.so'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-O2', '-shared', '-fPIC', '-I', str(ROOT / 'Core/Inc'),
                    str(ROOT / 'Core/Src/stima.c'), '-lm', '-o', str(lib)], check=True)
    c = ctypes.CDLL(str(lib))
    c.Stima_Reset.argtypes = [ctypes.POINTER(Stima)] + [ctypes.c_float]*3
    c.Stima_Predici.argtypes = [ctypes.POINTER(Stima), ctypes.c_float, ctypes.c_float]
    c.Stima_Correggi.argtypes = [ctypes.POINTER(Stima), ctypes.c_float, ctypes.c_float]
    c.Stima_Wrap.argtypes = [ctypes.c_float]
    c.Stima_Wrap.restype = ctypes.c_float
    return c


def costanti_uguali():
    """Le #define di stima.h devono coincidere con stimatore.toml."""
    header = (ROOT / 'Core/Inc/stima.h').read_text()
    p = ParametriStimatore.carica()
    for nome in ('k_g', 'k_a', 'k_d', 'l_theta', 'l_omega', 'l_x'):
        valore = float(re.search(rf'#define STIMA_{nome.upper()}\s+([0-9.eE+-]+)f', header)[1])
        assert valore == getattr(p, nome), f'{nome}: stima.h {valore} != toml {getattr(p, nome)}'


def confronta(c, seed):
    rng = np.random.default_rng(seed)
    py, cs = StimatoreStato(), Stima()
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
        th_m = math.remainder(theta_vero + rng.normal(0, 0.017), 2*math.pi)
        py.correggi(x_m, th_m)
        c.Stima_Correggi(ctypes.byref(cs), x_m, th_m)
        d_theta = abs(math.remainder(py.theta - cs.theta, 2*math.pi))
        errore = max(errore, d_theta, abs(py.omega - cs.omega)/10, abs(py.x - cs.x),
                     abs(py.velocita - cs.velocita))
        assert -math.pi < cs.theta <= math.pi
    return errore


def main():
    costanti_uguali()
    with tempfile.TemporaryDirectory(prefix='stima-c-') as cartella:
        c = compila(cartella)
        for a in (math.pi, -math.pi, 3*math.pi, 0.1, -7.):
            w = c.Stima_Wrap(a)
            assert -math.pi < w <= math.pi + 1e-6 and abs(math.remainder(w - a, 2*math.pi)) < 1e-5
        errori = [confronta(c, s) for s in range(20)]
    # float32 (C) contro float64 (Python): differenza massima su 20 x 60 s.
    assert max(errori) < 2e-3, errori
    print(f'OK: stimatore C = stima.py (differenza massima {max(errori):.2e}, 20 sequenze da 60 s)')


if __name__ == '__main__':
    main()
