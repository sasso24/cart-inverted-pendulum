"""Simula il pendolo con la rete SAC (o in manuale): swing-up o equilibrio
secondo la configurazione salvata con il modello."""
import argparse
from dataclasses import replace
from pathlib import Path
import json
import torch
from stable_baselines3 import SAC
from ambiente import ROOT
from addestra import check_contract, profilo_salvato
from configurazione import CONFIG_DEFAULT, Configurazione, configurazione_ambiente
from disturbi import ParametriDisturbi, crea_ambiente


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "modelli_asta20_basevecchia_storia1s_20s" / "migliore.zip")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--manual", action="store_true", help="Apri senza una rete addestrata; usa le frecce")
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument('--disturbi', type=Path, help='Profilo TOML delle perturbazioni')
    profile.add_argument('--nominale', action='store_true', help='Disabilita i disturbi salvati col modello')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--config', type=Path, default=CONFIG_DEFAULT,
                        help='Solo con --manual: episodio e motore da questo file')
    parser.add_argument('--smorzamento', type=float, metavar='N_M_S',
                        help='Smorzamento viscoso del perno fisso in N*m*s/rad '
                             '(default: XML, o estratto a ogni reset con i disturbi della rete)')
    parser.add_argument('--attrito', type=float, metavar='N_M',
                        help='Attrito coulombiano del perno fisso in N*m (default come sopra)')
    args = parser.parse_args()
    if not args.manual and not args.model.is_file():
        parser.error("Modello non trovato. Avvia prima Addestra.command o specifica --model.")
    torch.set_num_threads(1)
    agent = None
    cfg = Configurazione.carica(args.config) if args.manual else None
    if not args.manual:
        check_contract(args.model.parent)
        agent = SAC.load(args.model, device="cpu")
        cfg = configurazione_ambiente(args.model.parent)
    if args.nominale:
        disturbi = None
    elif args.disturbi:
        disturbi = ParametriDisturbi.carica(args.disturbi)
    elif args.manual:
        disturbi = None
    else:
        disturbi = profilo_salvato(args.model.parent)
    # Perno fisso: niente estrazione per episodio di quel valore (disturbi.py, _perno).
    if disturbi is not None and args.smorzamento is not None:
        disturbi = replace(disturbi, smorzamento_perno_min_nms=args.smorzamento,
                           smorzamento_perno_max_nms=args.smorzamento)
    if disturbi is not None and args.attrito is not None:
        disturbi = replace(disturbi, attrito_perno_max_nm=0.)
    env = crea_ambiente(disturbi, configurazione=cfg)
    for nome, valore in (('dof_damping', args.smorzamento), ('dof_frictionloss', args.attrito)):
        if valore is not None:
            getattr(env.model, nome)[1] = valore
            if hasattr(env, '_nominale'):
                env._nominale[nome][1] = valore   # ripristinato a ogni reset
    if args.headless:
        obs, initial = env.reset(seed=args.seed)
        total = 0.
        for _ in range(env.max_steps):
            action = [0.] if agent is None else agent.predict(obs, deterministic=True)[0]
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            if terminated or truncated:
                break
        print(json.dumps({"initial_angle_deg": initial["angle_deg"], "reward": total,
                          "terminated": terminated, **info,
                          "is_success": env.successo(terminated)}, indent=2))
    else:
        from finestra import run
        run(env, agent, seed=args.seed)
    env.close()


if __name__ == "__main__":
    main()
