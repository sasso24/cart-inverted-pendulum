"""Valutazione riproducibile con semi diversi da quelli della selezione modello."""
import argparse
import json
from pathlib import Path
import torch
from stable_baselines3 import SAC
from ambiente import ROOT
from addestra import evaluate_comparison, check_contract, profilo_salvato
from disturbi import ParametriDisturbi

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "modelli" / "migliore.zip")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--output", type=Path, default=ROOT / "risultati" / "valutazione.json")
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument('--disturbi', type=Path, help='Profilo di stress, anche per una policy nominale')
    profile.add_argument('--nominale', action='store_true', help='Valuta solo senza disturbi')
    args = parser.parse_args()
    if args.episodes <= 0:
        parser.error("Il numero di episodi deve essere positivo.")
    torch.set_num_threads(1)
    check_contract(args.model.parent)
    agent = SAC.load(args.model, device="cpu")
    disturbi = (None if args.nominale else ParametriDisturbi.carica(args.disturbi)
                if args.disturbi else profilo_salvato(args.model.parent))
    result = evaluate_comparison(agent, args.episodes, seed_start=2000, disturbi=disturbi)
    result["model"] = str(args.model.resolve())
    result["success_definition"] = "Intero episodio senza interruzioni con |angolo|<12 gradi, |x|<limite della corsa, |omega|<1 rad/s, |vx|<0.75 m/s"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(f"Successi: {result['success_rate']:.0%} su {args.episodes} prove; "
          f"ricompensa media: {result['mean_reward']:.1f}")
    if 'nominal' in result:
        print(f"Confronto nominale: successi {result['nominal']['success_rate']:.0%}; "
              f"ricompensa {result['nominal']['mean_reward']:.1f}")
    print(f"Dettagli: {args.output}")
