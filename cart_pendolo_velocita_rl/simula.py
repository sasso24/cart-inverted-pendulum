"""Simula il pendolo dall'alto con la rete SAC (o in manuale)."""
import argparse
from pathlib import Path
import json
import torch
from stable_baselines3 import SAC
from ambiente import ROOT
from addestra import check_contract, profilo_salvato
from disturbi import ParametriDisturbi, crea_ambiente


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "modelli" / "migliore.zip")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--manual", action="store_true", help="Apri senza una rete addestrata; usa le frecce")
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument('--disturbi', type=Path, help='Profilo TOML delle perturbazioni')
    profile.add_argument('--nominale', action='store_true', help='Disabilita i disturbi salvati col modello')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    if not args.manual and not args.model.is_file():
        parser.error("Modello non trovato. Avvia prima Addestra.command o specifica --model.")
    torch.set_num_threads(1)
    agent = None
    if not args.manual:
        check_contract(args.model.parent)
        agent = SAC.load(args.model, device="cpu")
    if args.nominale:
        disturbi = None
    elif args.disturbi:
        disturbi = ParametriDisturbi.carica(args.disturbi)
    elif args.manual:
        disturbi = None
    else:
        disturbi = profilo_salvato(args.model.parent)
    env = crea_ambiente(disturbi)
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
                          "is_success": not terminated and env.best_stable_steps == env.steps}, indent=2))
    else:
        from finestra import run
        run(env, agent, seed=args.seed)
    env.close()


if __name__ == "__main__":
    main()
