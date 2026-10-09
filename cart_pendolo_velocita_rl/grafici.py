"""Esporta traiettoria e grafici del controllo di velocita in Hz."""
import argparse
import csv
import os
from pathlib import Path
import tempfile
os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'stepper_hz_mpl'))
os.environ.setdefault('XDG_CACHE_HOME', str(Path(tempfile.gettempdir()) / 'stepper_hz_cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import torch
from stable_baselines3 import SAC
from ambiente import ROOT
from addestra import check_contract, profilo_salvato
from disturbi import ParametriDisturbi, crea_ambiente


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT / 'modelli/migliore.zip')
    parser.add_argument('--output', type=Path, default=ROOT / 'risultati')
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument('--disturbi', type=Path)
    profile.add_argument('--nominale', action='store_true')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    torch.set_num_threads(1)
    check_contract(args.model.parent)
    agent = SAC.load(args.model, device='cpu')
    disturbi = (None if args.nominale else ParametriDisturbi.carica(args.disturbi)
                if args.disturbi else profilo_salvato(args.model.parent))
    env = crea_ambiente(disturbi)
    obs, info = env.reset(seed=args.seed, options={'exact': True})
    rows = [[0., info['angle_deg'], info['cart_x'], 0., 0., 0., 0.]]
    for _ in range(env.max_steps):
        action, _ = agent.predict(obs, deterministic=True)
        obs, _, done, truncated, info = env.step(action)
        rows.append([float(env.data.time), info['angle_deg'], info['cart_x'],
                     info['requested_frequency_hz'], info['motor_frequency_hz'],
                     info['command_velocity_m_s'], info['cart_velocity_m_s']])
        if done or truncated:
            break
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / 'traiettoria.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'angle_deg', 'cart_x_m', 'requested_frequency_hz',
                         'motor_frequency_hz', 'command_velocity_m_s', 'cart_velocity_m_s'])
        writer.writerows(rows)
    t, angle, x, requested, hz, target, velocity = np.asarray(rows).T
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle('Pendolo con stepper — SAC velocita in Hz')
    axes[0, 0].plot(t, np.abs(angle))
    axes[0, 0].axhspan(0, 12, color='green', alpha=.12)
    axes[0, 0].set(title='Distanza dalla verticale', ylabel='gradi')
    axes[0, 1].plot(t, x)
    axes[0, 1].axhline(env.limite, color='red', linestyle='--')
    axes[0, 1].axhline(-env.limite, color='red', linestyle='--')
    axes[0, 1].set(title='Posizione carrello', ylabel='m')
    axes[1, 0].step(t, requested, where='post', label='Richiesta rete', alpha=.6)
    axes[1, 0].plot(t, hz, label='Comando dopo rampa e arresti')
    axes[1, 0].set(title='Frequenza STEP firmata', ylabel='Hz')
    axes[1, 0].legend()
    axes[1, 1].plot(t, target, label='Comandata')
    axes[1, 1].plot(t, velocity, label='Simulata')
    axes[1, 1].set(title='Velocita carrello', ylabel='m/s')
    axes[1, 1].legend()
    for ax in axes.flat:
        ax.set_xlabel('Tempo (s)')
        ax.grid(alpha=.2)
    fig.savefig(args.output / 'controllo_stepper.png', dpi=150)
    plt.close(fig)
    env.close()
    print(f'Salvati traiettoria.csv e controllo_stepper.png in {args.output}')


if __name__ == '__main__':
    main()
