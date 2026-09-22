"""Esporta una traiettoria riproducibile e grafici senza richiedere una finestra."""
import csv
import os
from pathlib import Path
import tempfile
os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'stepper_mpl'))
os.environ.setdefault('XDG_CACHE_HOME', str(Path(tempfile.gettempdir()) / 'stepper_cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import torch
from stable_baselines3 import SAC
from ambiente import ROOT, CartPendoloEnv
from addestra import check_contract


def main():
    torch.set_num_threads(1)
    check_contract(ROOT / 'modelli')
    agent = SAC.load(ROOT / 'modelli/migliore.zip', device='cpu')
    env = CartPendoloEnv()
    obs, _ = env.reset(options={'exact': True})
    rows = [[0., 180., 0., 0., 0., 0., 0.]]
    # Registriamo sia la stima da passi sia la verità fisica, per quantificare
    # l'errore del modello equivalente invece di confondere le due posizioni.
    for _ in range(env.max_steps):
        action, _ = agent.predict(obs, deterministic=True)
        obs, _, done, truncated, info = env.step(action)
        rows.append([env.data.time, info['angle_deg'], info['cart_x'], info['estimated_x'],
                     float(action[0])*env.stepper.p.accelerazione_max,
                     info['command_velocity'], env.data.ctrl[0]])
        if done or truncated: break
    folder = ROOT / 'risultati'
    folder.mkdir(exist_ok=True)
    with (folder / 'traiettoria.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s','angle_deg','real_x_m','estimated_x_m',
                         'requested_acceleration_m_s2','command_velocity_m_s','equivalent_force_N'])
        writer.writerows(rows)
    t, angle, x, estimated, acceleration, velocity, force = np.asarray(rows).T
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle('Pendolo con stepper — SAC, modello nominale')
    axes[0, 0].plot(t, np.abs(angle))
    axes[0, 0].axhspan(0, 12, color='green', alpha=.12)
    axes[0, 0].set(title='Swing-up dal basso', ylabel='Distanza dalla verticale (°)')
    axes[0, 1].plot(t, x, label='Reale simulata')
    axes[0, 1].plot(t, estimated, '--', label='Conteggio impulsi')
    axes[0, 1].set(title='Posizione del carrello', ylabel='m')
    axes[0, 1].legend()
    axes[1, 0].plot(t, (estimated-x)*1000)
    axes[1, 0].set(title='Errore di inseguimento', ylabel='mm')
    axes[1, 1].plot(t, acceleration)
    axes[1, 1].set(title='Accelerazione richiesta dalla policy', ylabel='m/s²')
    for ax in axes.flat:
        ax.set_xlabel('Tempo (s)')
        ax.grid(alpha=.2)
    fig.savefig(folder / 'controllo_stepper.png', dpi=150)
    plt.close(fig)
    env.close()
    print('Salvati risultati/traiettoria.csv e risultati/controllo_stepper.png')


if __name__ == '__main__':
    main()
