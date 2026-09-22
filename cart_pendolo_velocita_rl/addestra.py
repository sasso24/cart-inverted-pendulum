"""Addestra un agente SAC da zero o riprende un addestramento salvato."""
import hashlib
from dataclasses import asdict
from stepper import ParametriStepper
import argparse
import csv
import json
from pathlib import Path
import time
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from ambiente import CartPendoloEnv, ROOT


def contract():
    """Firma del modello E del codice: impedisce riprese con dinamica cambiata."""
    names = ('cart_pendolo.xml', 'ambiente.py', 'stepper.py', 'stepper.toml')
    return {"sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in names},
            "stepper": asdict(ParametriStepper.carica()), "observation_size": 5,
            "action": "signed_normalized_step_frequency", "control_hz": 50}


def check_contract(folder):
    path = folder / 'config.json'
    if not path.exists() or json.loads(path.read_text()).get('environment') != contract():
        raise ValueError('Checkpoint incompatibile con ambiente/parametri. Addestrare in una nuova cartella.')


def evaluate(agent, episodes=5, seed_start=1000):
    """Policy deterministica su semi fissi; la selezione non usa i semi di test."""
    env = CartPendoloEnv()
    rows = []
    for index in range(episodes):
        obs, _ = env.reset(seed=seed_start + index)
        total = 0.
        for _ in range(env.max_steps):
            action, _ = agent.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            if terminated or truncated:
                break
        rows.append({"seed": seed_start + index, "reward": total,
                     "duration_s": float(env.data.time), "terminated": terminated, **info})
    env.close()
    return {"episodes": episodes, "mean_reward": sum(r["reward"] for r in rows) / episodes,
            "success_rate": sum(r["is_success"] for r in rows) / episodes, "details": rows}


class Progress(BaseCallback):
    def __init__(self, folder, frequency):
        super().__init__()
        self.folder, self.frequency = folder, frequency
        self.started = time.monotonic()
        self.best = float("-inf")
        if (folder / "best_metrics.json").exists():
            self.best = json.loads((folder / "best_metrics.json").read_text())["mean_reward"]

    def _on_step(self):
        # Valutare costa: eseguiamo episodi completi solo ogni frequency decisioni.
        if self.n_calls % self.frequency:
            return True
        metrics = evaluate(self.model)
        metrics["timesteps"] = self.num_timesteps
        metrics["elapsed_seconds_this_run"] = time.monotonic() - self.started
        history = self.folder / "progresso.csv"
        exists = history.exists()
        with history.open("a", newline="") as stream:
            writer = csv.writer(stream)
            if not exists:
                writer.writerow(["steps", "reward", "success_rate", "elapsed_s_this_run"])
            writer.writerow([self.num_timesteps, metrics["mean_reward"],
                             metrics["success_rate"], metrics["elapsed_seconds_this_run"]])
        self.model.save(self.folder / "ultimo")
        if metrics["mean_reward"] > self.best:
            self.best = metrics["mean_reward"]
            self.model.save(self.folder / "migliore")
            (self.folder / "best_metrics.json").write_text(json.dumps(metrics, indent=2))
        print(f"Passi {self.num_timesteps:,} | ricompensa {metrics['mean_reward']:.1f} "
              f"| successi {metrics['success_rate']:.0%} | "
              f"tempo {metrics['elapsed_seconds_this_run']/60:.1f} min", flush=True)
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=400000, help="Nuovi passi di addestramento")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "modelli")
    parser.add_argument("--eval-every", type=int, default=10000)
    args = parser.parse_args()
    if args.steps <= 0 or args.eval_every <= 0:
        parser.error("Passi e frequenza devono essere positivi.")
    args.output.mkdir(parents=True, exist_ok=True)
    if not args.resume and any(args.output.iterdir()):
        parser.error("Esiste un addestramento: usa --resume oppure --output con una nuova cartella.")
    torch.set_num_threads(1)
    env = Monitor(CartPendoloEnv())
    if args.resume:
        check_contract(args.output)
        agent = SAC.load(args.output / "ultimo", env=env, device="cpu")
        buffer = args.output / "esperienze.pkl"
        if buffer.exists():
            agent.load_replay_buffer(buffer)
        else:
            agent.learning_starts = agent.num_timesteps + 2000
            print("Buffer non disponibile: raccolgo nuove esperienze prima di aggiornare la rete.", flush=True)
    else:
        # Due strati da 128 ReLU: stessa taglia di base del progetto precedente,
        # ma cinque ingressi. SAC usa due critici SOLO in addestramento; il firmware
        # eseguirà l'attore deterministico (media seguita da tanh).
        # Una ottimizzazione ogni quattro transizioni limita il costo CPU.
        agent = SAC("MlpPolicy", env, seed=args.seed, device="cpu", verbose=0,
                    learning_rate=3e-4, buffer_size=300000, learning_starts=3000,
                    batch_size=128, gamma=0.99, tau=0.005, train_freq=4,
                    gradient_steps=1, ent_coef="auto_0.1", policy_kwargs={"net_arch": [128, 128]})
        (args.output / "config.json").write_text(json.dumps({"algorithm": "SAC", "seed": args.seed,
            "initial_state": "downward with small random perturbations", "control_hz": 50,
            "max_episode_seconds": 20, "environment": contract()}, indent=2))
    print("Addestramento SAC: tutti gli episodi partono dal basso. Ctrl+C salva ed esce.", flush=True)
    # Salviamo anche il replay buffer per poter riprendere senza perdere esperienze.
    try:
        agent.learn(total_timesteps=args.steps, callback=Progress(args.output, args.eval_every),
                    reset_num_timesteps=not args.resume)
    except KeyboardInterrupt:
        print("Interruzione richiesta: salvo lo stato corrente.", flush=True)
    finally:
        agent.save(args.output / "ultimo")
        agent.save_replay_buffer(args.output / "esperienze.pkl")
        env.close()
    print("Salvataggio completato.", flush=True)


if __name__ == "__main__":
    main()
