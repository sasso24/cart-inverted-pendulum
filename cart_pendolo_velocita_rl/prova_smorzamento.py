"""Sweep dello smorzamento del perno: la rete aggancia anche se il perno frena poco?

Diagnosi del 24/10: con il damping 0.01 dell'XML storico le reti non passano mai
sopra la verticale; con un perno reale che frena meno il pendolo arriva in alto
troppo veloce, gira e non aggancia. Qui ogni episodio usa uno smorzamento del
perno FISSO (sostituisce quello dell'XML e quello estratto da [disturbi]), con
attrito coulombiano e resistenza dell'aria fissi (default 0). Lo stimatore resta
quello salvato con la rete (come sul firmware).

  nominale  stato vero, nessun disturbo ne stimatore
  disturbi  profilo [disturbi] e [sensori] della rete, stimatore della rete

    python prova_smorzamento.py --model modelli_asta20_basevecchia_storia1s_20s/migliore.zip
    python prova_smorzamento.py --model a/migliore.zip b/migliore.zip --smorzamenti 0.01 0.001 --attrito 0.001

Per ogni smorzamento: successi, passaggi sopra la verticale (entra nella meta alta
da un lato ed esce dall'altro) e primo arrivo in alto (|angolo| < stabile_angolo_gradi).
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np

SMORZAMENTI = (0.012, 0.01, 0.005, 0.0025, 0.001, 0.0003)
CONDIZIONI = ('nominale', 'disturbi')


def episodio(agent, env, seed, smorzamento, attrito, aria):
    obs, _ = env.reset(seed=seed)
    env.model.dof_damping[1] = smorzamento
    env.model.dof_frictionloss[1] = attrito
    if hasattr(env, 'resistenza_aria'):
        env.resistenza_aria = aria
    soglia = math.radians(env.ep.stabile_angolo_gradi)
    passaggi, lato, primo_arrivo, terminato = 0, 0, None, False
    for _ in range(env.max_steps):
        azione, _ = agent.predict(obs, deterministic=True)
        obs, _, terminato, troncato, info = env.step(azione)
        a = math.remainder(float(env.data.qpos[1]), 2*math.pi)
        if abs(a) < math.pi/2:
            if lato == 0:
                lato = 1 if a > 0 else -1     # lato da cui entra nella meta alta
        elif lato:
            passaggi += (1 if a > 0 else -1) != lato
            lato = 0
        if primo_arrivo is None and abs(a) < soglia:
            primo_arrivo = float(env.data.time)
        if terminato or troncato:
            break
    return {'seed': seed, 'successo': bool(env.successo(terminato)), 'passaggi': passaggi,
            'primo_arrivo_s': primo_arrivo, 'motivo': info['termination_reason']}


def prova(modello, condizione, smorzamento, attrito, aria, episodi, seed_start, durata):
    import torch
    from stable_baselines3 import SAC
    from configurazione import configurazione_modello
    from disturbi import crea_ambiente
    torch.set_num_threads(1)
    cfg = configurazione_modello(Path(modello).parent)
    if durata is not None:
        cfg = replace(cfg, episodio=replace(cfg.episodio, durata_s=durata))
    agent = SAC.load(modello, device='cpu')
    env = crea_ambiente(None if condizione == 'nominale' else cfg.disturbi, configurazione=cfg)
    if condizione == 'nominale':
        env.sensori = None
    righe = [episodio(agent, env, seed_start + i, smorzamento, attrito, aria) for i in range(episodi)]
    arrivi = [r['primo_arrivo_s'] for r in righe if r['primo_arrivo_s'] is not None]
    return {'modello': str(modello), 'condizione': condizione, 'smorzamento_nms': smorzamento,
            'successi': sum(r['successo'] for r in righe), 'episodi': episodi,
            'passaggi_medi': float(np.mean([r['passaggi'] for r in righe])),
            'primo_arrivo_mediano_s': float(np.median(arrivi)) if arrivi else None,
            'mai_in_alto': episodi - len(arrivi), 'dettagli': righe}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', type=Path, nargs='+', required=True)
    parser.add_argument('--condizioni', nargs='+', default=list(CONDIZIONI), choices=CONDIZIONI)
    parser.add_argument('--smorzamenti', type=float, nargs='+', default=list(SMORZAMENTI),
                        help='Smorzamento viscoso del perno in N*m*s/rad')
    parser.add_argument('--attrito', type=float, default=0., help='Attrito coulombiano del perno in N*m')
    parser.add_argument('--aria', type=float, default=0., help='Resistenza dell aria in N*m*s^2 (solo disturbi)')
    parser.add_argument('--episodes', type=int, default=20)
    parser.add_argument('--seed-start', type=int, default=3000)
    parser.add_argument('--durata', type=float, default=20., help='Durata in s (default 20)')
    parser.add_argument('--processi', type=int, default=4)
    parser.add_argument('--output', type=Path, default=Path('risultati/prova_smorzamento.json'))
    args = parser.parse_args()
    lavori = [(str(m), c, s, args.attrito, args.aria, args.episodes, args.seed_start, args.durata)
              for m in args.model for s in args.smorzamenti for c in args.condizioni]
    with ProcessPoolExecutor(args.processi) as pool:
        risultati = list(pool.map(prova, *zip(*lavori)))
    for r in risultati:
        arrivo = r['primo_arrivo_mediano_s']
        print(f"{r['modello']} | {r['condizione']:8s} | damping {r['smorzamento_nms']:.4f} | "
              f"{r['successi']:2d}/{r['episodi']} | passaggi {r['passaggi_medi']:5.1f} | "
              f"primo arrivo {'-' if arrivo is None else f'{arrivo:.1f} s'}"
              f"{f' (mai in alto {r["mai_in_alto"]})' if r['mai_in_alto'] else ''}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'attrito_nm': args.attrito, 'aria_nms2': args.aria,
                                       'risultati': risultati}, indent=2))


if __name__ == '__main__':
    main()
