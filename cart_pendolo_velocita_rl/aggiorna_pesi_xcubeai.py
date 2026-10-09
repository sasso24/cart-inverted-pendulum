"""Copia i pesi di una rete SAC nel firmware X-CUBE-AI gia generato.

Il codice generato da X-CUBE-AI (network.c) resta invariato: cambia solo
l'array dei pesi in network_data_params.c. Vale per la rete 6-128-128-1
generata con `--compression none`: float32 in ordine
W0[128x6], b0, W1[128x128], b1, W2[1x128], b2, piu un float di riempimento.
L'ordine e stato ricavato e verificato decodificando i pesi della rete
precedente (identici bit per bit al checkpoint esportato il 29/09).

    python aggiorna_pesi_xcubeai.py --model modelli_disturbi/migliore.zip \
        --params ../Prova_controllo/X-CUBE-AI/App/network_data_params.c

Per rigenerare tutto con gli strumenti ST si puo invece importare
export_stm32/pendolo_actor.onnx in STM32CubeMX (X-CUBE-AI).
"""
import argparse
from pathlib import Path
import re

import numpy as np
import torch
from stable_baselines3 import SAC

ROOT = Path(__file__).resolve().parent
NOME_ARRAY = 's_network_weights_array_u64'


def pesi_rete(model_path):
    actor = SAC.load(model_path, device='cpu').actor
    layers = [actor.latent_pi[0], actor.latent_pi[2], actor.mu]
    if [(l.in_features, l.out_features) for l in layers] != [(6, 128), (128, 128), (128, 1)]:
        raise ValueError('Serve una rete 6-128-128-1.')
    parti = []
    for layer in layers:
        parti += [layer.weight.detach().numpy().ravel(), layer.bias.detach().numpy()]
    return np.concatenate(parti).astype('<f4')


def leggi_array(testo):
    inizio = testo.index(NOME_ARRAY)
    corpo = testo[testo.index('{', inizio) + 1:testo.index('};', inizio)]
    valori = [int(h, 16) for h in re.findall(r'0x([0-9a-fA-F]+)U', corpo)]
    return inizio, np.frombuffer(np.array(valori, dtype='<u8').tobytes(), dtype='<f4')


def scrivi_array(testo, pesi):
    inizio, attuali = leggi_array(testo)
    if len(attuali) != len(pesi) + 1:
        raise ValueError(f'Dimensione diversa: firmware {len(attuali)} float, rete {len(pesi)}+1.')
    dati = np.concatenate([pesi, np.zeros(1, '<f4')])
    parole = np.frombuffer(dati.tobytes(), dtype='<u8')
    righe = []
    for k in range(0, len(parole), 4):
        # Stesso formato del generatore ST: esadecimale minimo, virgola finale.
        righe.append('  ' + ', '.join(f'0x{int(w):x}U' for w in parole[k:k+4]) + ',')
    apertura = testo.index('{', inizio) + 1
    chiusura = testo.index('};', inizio)
    return testo[:apertura] + '\n' + '\n'.join(righe) + '\n' + testo[chiusura:]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', type=Path, default=ROOT / 'modelli_disturbi/migliore.zip')
    parser.add_argument('--params', type=Path, default=ROOT.parent / 'Prova_controllo'
                        / 'X-CUBE-AI/App/network_data_params.c')
    args = parser.parse_args()
    pesi = pesi_rete(args.model)
    # newline='' conserva le terminazioni di riga originali (LF o CRLF).
    with args.params.open(newline='') as stream:
        testo = scrivi_array(stream.read(), pesi)
    _, riletti = leggi_array(testo)
    if not np.array_equal(riletti[:len(pesi)], pesi):
        raise RuntimeError('Verifica fallita dopo la scrittura.')
    with args.params.open('w', newline='') as stream:
        stream.write(testo)
    print(f'Aggiornati {len(pesi)} pesi in {args.params}')


if __name__ == '__main__':
    main()
