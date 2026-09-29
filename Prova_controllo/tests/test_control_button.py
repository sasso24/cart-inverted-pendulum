#!/usr/bin/env python3
"""Esegue su host il codice applicativo reale con timer/GPIO simulati.
Non sostituisce la misura su scheda della latenza EXTI -> STEP.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'Core/Src/main.c').read_text()

def block(name):
    return source.split(f'/* USER CODE BEGIN {name} */', 1)[1].split(
        f'/* USER CODE END {name} */', 1)[0]

with tempfile.TemporaryDirectory(prefix='stm32-button-') as directory:
    path = Path(directory)
    application = '\n'.join(block(name) for name in ('PD', 'PV', '0', '4'))
    (path / 'application.inc').write_text(application)
    executable = path / 'test_button'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(path), str(ROOT / 'tests/test_control_button.c'),
                    '-lm', '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
