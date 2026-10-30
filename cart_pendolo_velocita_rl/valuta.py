"""Valutazione riproducibile con semi diversi da quelli della selezione modello."""
import argparse
import json
from pathlib import Path
import torch
from stable_baselines3 import SAC
from ambiente import ROOT
from addestra import evaluate_comparison, check_contract, profilo_salvato
from disturbi import ParametriDisturbi
from configurazione import configurazione_ambiente

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "modelli_asta20_basevecchia_storia1s_20s" / "migliore.zip")
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
    cfg = configurazione_ambiente(args.model.parent)
    result = evaluate_comparison(agent, args.episodes, seed_start=2000, disturbi=disturbi,
                                 configurazione=cfg)
    result["model"] = str(args.model.resolve())
    ep = (cfg or __import__('configurazione').Configurazione()).episodio
    result["success_definition"] = (
        f"In equilibrio senza interruzioni da {ep.tempo_max_salita_s:g} s alla fine "
        f"(|angolo|<{ep.stabile_angolo_gradi:g} gradi, |x|<limite della corsa, "
        f"|omega|<{ep.stabile_omega_rad_s:g} rad/s, |vx|<{ep.stabile_velocita_m_s:g} m/s)")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(f"Successi: {result['success_rate']:.0%} su {args.episodes} prove; "
          f"ricompensa media: {result['mean_reward']:.1f}")
    if 'nominal' in result:
        print(f"Confronto nominale: successi {result['nominal']['success_rate']:.0%}; "
              f"ricompensa {result['nominal']['mean_reward']:.1f}")
    print(f"Dettagli: {args.output}")
