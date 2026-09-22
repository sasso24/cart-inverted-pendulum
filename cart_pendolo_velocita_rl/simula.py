"""Carica l'agente SAC salvato e controlla il pendolo partendo dal basso."""
import argparse
from pathlib import Path
import json
import torch
from stable_baselines3 import SAC
from ambiente import ROOT, CartPendoloEnv
from addestra import check_contract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "modelli" / "migliore.zip")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--manual", action="store_true", help="Apri senza una rete addestrata; usa le frecce")
    args = parser.parse_args()
    if not args.manual and not args.model.is_file():
        parser.error("Modello non trovato. Avvia prima Addestra.command o specifica --model.")
    torch.set_num_threads(1)
    agent = None
    if not args.manual:
        check_contract(args.model.parent)
        agent = SAC.load(args.model, device="cpu")
    env = CartPendoloEnv()
    if args.headless:
        obs, _ = env.reset(options={"exact": True})
        total = 0.
        for _ in range(env.max_steps):
            action = [0.] if agent is None else agent.predict(obs, deterministic=True)[0]
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            if terminated or truncated:
                break
        print(json.dumps({"initial_angle_deg": 180, "reward": total,
                          "terminated": terminated, **info}, indent=2))
    else:
        from finestra import run
        run(env, agent)
    env.close()


if __name__ == "__main__":
    main()
