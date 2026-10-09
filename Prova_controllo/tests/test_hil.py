"""Prova su host del protocollo HIL senza scheda: il codice HIL reale di main.c
(HIL_MODE=1) gira sul PC, la UART e' uno pseudo-terminale e la rete X-CUBE-AI
e' sostituita da un MLP float32 in C con i pesi letti da network_data_params.c.
Poi hil.py viene eseguito contro questo "finto STM32".

python3 Prova_controllo/tests/test_hil.py   (dalla radice, ambiente Python del progetto RL)
Solo Linux/macOS (usa pty). Non misura i tempi reali della scheda.
"""
import json
import os
from pathlib import Path
import pty
import re
import subprocess
import sys
import tempfile
import tty

ROOT = Path(__file__).resolve().parents[1]
PY = ROOT.parent / 'cart_pendolo_velocita_rl'
sys.path.insert(0, str(PY))
from aggiorna_pesi_xcubeai import leggi_array  # noqa: E402

sorgente_test = (ROOT / 'tests/test_controllo.py').read_text()
MOCK = re.search(r"MOCK = r'''(.*?)'''", sorgente_test, re.S)[1]
vecchia_run = MOCK[MOCK.index('static ai_i32 ai_network_run'):]
vecchia_run = vecchia_run[:vecchia_run.index('\n}\n') + 3]
MOCK = MOCK.replace(vecchia_run, r'''
#include "pesi_mlp.h"
static ai_i32 ai_network_run(ai_handle h, const ai_buffer *i, ai_buffer *o) {
    const float *x = (const float *)i[0].data; float h1[128], h2[128];
    const float *W0=PESI, *b0=W0+768, *W1=b0+128, *b1=W1+16384, *W2=b1+128, *b2=W2+128;
    for (int r=0;r<128;r++){ float s=b0[r]; for(int c=0;c<6;c++) s+=W0[r*6+c]*x[c]; h1[r]=s>0?s:0; }
    for (int r=0;r<128;r++){ float s=b1[r]; for(int c=0;c<128;c++) s+=W1[r*128+c]*h1[c]; h2[r]=s>0?s:0; }
    float s=b2[0]; for(int c=0;c<128;c++) s+=W2[c]*h2[c];
    ((float *)o[0].data)[0]=tanhf(s);
    return 1;
}
''')
MOCK += r'''
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#define HAL_MAX_DELAY 0xFFFFFFFFU
static int uart_fd = -1;
UART_HandleTypeDef huart2;
static HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *h, uint8_t *b, uint16_t n, uint32_t t) {
    for (uint16_t k = 0; k < n; ) {
        struct pollfd p = {uart_fd, POLLIN, 0};
        if (poll(&p, 1, t == HAL_MAX_DELAY ? -1 : (int)t) <= 0) return HAL_TIMEOUT;
        ssize_t r = read(uart_fd, b + k, n - k);
        if (r <= 0) exit(0);  /* PC disconnesso: fine della prova */
        k += (uint16_t)r;
    }
    return HAL_OK;
}
static HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *b, uint16_t n, uint32_t t) {
    return write(uart_fd, b, n) == n ? HAL_OK : HAL_ERROR;
}
static void HAL_GPIO_TogglePin(int port, int pin) { pins[pin] = !pins[pin]; }
static uint32_t SystemCoreClock = 180000000U;
static struct { uint32_t DEMCR; } core_debug;
static struct { uint32_t CTRL, CYCCNT; } dwt;
#define CoreDebug (&core_debug)
#define DWT (&dwt)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
'''
MAIN = r'''
int main(int argc, char **argv) {
    uart_fd = atoi(argv[1]);
    htim2.Instance=&timer2; htim3.Instance=&timer3; htim3.Channel=HAL_TIM_ACTIVE_CHANNEL_1;
    assert(Rete_Init());
    Hil_Esegui();
}
'''


def section(text, name):
    return text.split(f'/* USER CODE BEGIN {name} */', 1)[1].split(f'/* USER CODE END {name} */', 1)[0]


def main():
    source = (ROOT / 'Core/Src/main.c').read_text()
    _, pesi = leggi_array((ROOT / 'X-CUBE-AI/App/network_data_params.c').read_text())
    stima = ((ROOT / 'Core/Inc/stima.h').read_text()
             + (ROOT / 'Core/Src/stima.c').read_text().replace('#include "stima.h"', ''))
    unit = ('#define HIL_MODE 1\n' + MOCK + stima + section(source, 'PD')
            + '\nTIM_HandleTypeDef htim2, htim3;\n')
    for name in ('PV', 'PFP', '0', '4'):
        unit += section(source, name)
    unit += MAIN
    with tempfile.TemporaryDirectory(prefix='hil-test-') as tmp:
        tmp = Path(tmp)
        (tmp / 'pesi_mlp.h').write_text(
            'static const float PESI[] = {' + ','.join(f'{v!r}f' for v in pesi.tolist()) + '};\n')
        (tmp / 'scheda.c').write_text(unit)
        subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-O2', '-Wall', '-Wextra',
                        '-Wno-unused-parameter', '-Wno-unused-function', '-I', str(tmp),
                        str(tmp / 'scheda.c'), '-lm', '-o', str(tmp / 'scheda')], check=True)
        master, slave = pty.openpty()
        tty.setraw(slave)
        scheda = subprocess.Popen([str(tmp / 'scheda'), str(master)], pass_fds=[master])
        try:
            porta = os.ttyname(slave)
            out = tmp / 'risultati'
            subprocess.run([sys.executable, str(PY / 'hil.py'), '--porta', porta,
                            '--episodi', '3', '--secondi', '10', '--output', str(out)],
                           check=True, cwd=PY, timeout=900)
            riepilogo = json.loads(next(out.glob('hil_*.json')).read_text())
        finally:
            scheda.kill()
            os.close(master)
            os.close(slave)
    assert riepilogo['successi'] == riepilogo['episodi'], riepilogo
    assert riepilogo['diff_obs_max'] < 1e-4, riepilogo['diff_obs_max']
    assert riepilogo['diff_azione_max'] < 1e-4, riepilogo['diff_azione_max']
    print('OK: protocollo HIL, stimatore C e pesi del firmware coerenti con PC/PyTorch')


if __name__ == '__main__':
    main()
