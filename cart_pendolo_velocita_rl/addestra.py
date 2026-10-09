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
from ambiente import ROOT
from disturbi import ParametriDisturbi, crea_ambiente


def contract(disturbi=None):
    """Firma del modello E del codice: impedisce riprese con dinamica cambiata."""
    names = ('cart_pendolo.xml', 'ambiente.py', 'stepper.py', 'stepper.toml')
    result = {"sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in names},
              "stepper": asdict(ParametriStepper.carica()), "observation_size": 6,
              "action": "signed_normalized_step_frequency", "control_hz": 50}
    if disturbi is not None:
        for name in ('disturbi.py', 'stima.py', 'stimatore.toml'):
            result['sha256'][name] = hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
        result['disturbi'] = json.loads(json.dumps(asdict(disturbi)))
    return result


def profilo_salvato(folder):
    config = json.loads((Path(folder) / 'config.json').read_text())
    values = config.get('environment', {}).get('disturbi')
    return None if values is None else ParametriDisturbi(**values)


def check_contract(folder):
    path = folder / 'config.json'
    if not path.exists() or json.loads(path.read_text()).get('environment') != contract(profilo_salvato(folder)):
        raise ValueError('Checkpoint incompatibile con ambiente/parametri. Addestrare in una nuova cartella.')


def evaluate(agent, episodes=5, seed_start=1000, disturbi=None):
    """Policy deterministica su semi fissi; la selezione non usa i semi di test."""
    if episodes <= 0:
        raise ValueError('episodes deve essere positivo.')
    env = crea_ambiente(disturbi)
    rows = []
    for index in range(episodes):
        obs, _ = env.reset(seed=seed_start + index)
        total = 0.
        max_angle = max_x = 0.
        for _ in range(env.max_steps):
            action, _ = agent.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            max_angle = max(max_angle, abs(info["angle_deg"]))
            max_x = max(max_x, abs(info["cart_x"]))
            if terminated or truncated:
                break
        # Tre secondi iniziali non bastano: una caduta successiva e un fallimento.
        sustained = (not terminated and truncated
                     and env.best_stable_steps == env.steps)
        rows.append({"seed": seed_start + index, "reward": total,
                     "duration_s": float(env.data.time), "terminated": terminated, **info,
                     "three_second_success": info["is_success"],
                     "is_success": sustained, "max_angle_deg": max_angle, "max_cart_x_m": max_x})
    env.close()
    return {"episodes": episodes, "mean_reward": sum(r["reward"] for r in rows) / episodes,
            "success_rate": sum(r["is_success"] for r in rows) / episodes,
            "worst_stable_seconds": min(r['stable_seconds'] for r in rows),
            "mean_stable_seconds": sum(r['stable_seconds'] for r in rows) / episodes,
            "details": rows}


def evaluate_comparison(agent, episodes=5, seed_start=1000, disturbi=None):
    result = evaluate(agent, episodes, seed_start, disturbi)
    result['selection_score'] = result['worst_stable_seconds']
    result['selection_reward'] = result['mean_reward']
    if disturbi is not None:
        result['nominal'] = evaluate(agent, episodes, seed_start)
        # Evita di selezionare una rete che migliora sotto disturbi ma peggiora
        # molto sul nominale. Le due suite hanno gli stessi stati iniziali.
        result['selection_score'] = min(result['worst_stable_seconds'], result['nominal']['worst_stable_seconds'])
        result['selection_reward'] = min(result['mean_reward'], result['nominal']['mean_reward'])
        result['disturbi'] = json.loads(json.dumps(asdict(disturbi)))
    return result


class Progress(BaseCallback):
    def __init__(self, folder, frequency, disturbi=None):
        super().__init__()
        self.folder, self.frequency = folder, frequency
        self.disturbi = disturbi
        self.started = time.monotonic()
        self.best = (float("-inf"), float("-inf"))
        if (folder / "best_metrics.json").exists():
            previous = json.loads((folder / "best_metrics.json").read_text())
            if previous.get('selection_version') == 3:
                self.best = (previous['selection_score'], previous['selection_reward'])

    def _on_step(self):
        # Valutare costa: eseguiamo episodi completi solo ogni frequency decisioni.
        if self.n_calls % self.frequency:
            return True
        metrics = evaluate_comparison(self.model, disturbi=self.disturbi)
        metrics["selection_version"] = 3
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
        # A parita di durata stabile (spesso tutti i 20 s) vince la ricompensa
        # peggiore tra nominale e disturbato: evita di tenere la prima rete appena sufficiente.
        candidate = (metrics["selection_score"], metrics["selection_reward"])
        if candidate > self.best:
            self.best = candidate
            self.model.save(self.folder / "migliore")
            (self.folder / "best_metrics.json").write_text(json.dumps(metrics, indent=2))
        print(f"Passi {self.num_timesteps:,} | ricompensa {metrics['mean_reward']:.1f} "
              f"| successi {metrics['success_rate']:.0%} | "
              f"tempo {metrics['elapsed_seconds_this_run']/60:.1f} min", flush=True)
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=450000, help="Nuovi passi di addestramento")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "modelli")
    parser.add_argument("--eval-every", type=int, default=10000)
    parser.add_argument('--parti-da', type=Path, help='Nuovo addestramento che parte dai pesi di una rete .zip')
    parser.add_argument('--disturbi', type=Path, help='Profilo TOML; senza opzione il nuovo training e nominale')
    args = parser.parse_args()
    if args.steps <= 0 or args.eval_every <= 0:
        parser.error("Passi e frequenza devono essere positivi.")
    args.output.mkdir(parents=True, exist_ok=True)
    if args.resume and args.parti_da:
        parser.error("Usa --resume oppure --parti-da, non entrambi.")
    if not args.resume and any(args.output.iterdir()):
        parser.error("Esiste un addestramento: usa --resume oppure --output con una nuova cartella.")
    disturbi = ParametriDisturbi.carica(args.disturbi) if args.disturbi else None
    if args.resume:
        check_contract(args.output)
        saved = profilo_salvato(args.output)
        if args.disturbi and disturbi != saved:
            parser.error('Profilo disturbi diverso dal checkpoint: usare una nuova cartella.')
        disturbi = saved
    torch.set_num_threads(1)
    env = Monitor(crea_ambiente(disturbi))
    if args.resume:
        agent = SAC.load(args.output / "ultimo", env=env, device="cpu")
        buffer = args.output / "esperienze.pkl"
        if buffer.exists():
            agent.load_replay_buffer(buffer)
        else:
            agent.learning_starts = agent.num_timesteps + 2000
            print("Buffer non disponibile: raccolgo nuove esperienze prima di aggiornare la rete.", flush=True)
    elif args.parti_da:
        # Pesi e coefficiente di entropia della rete indicata; buffer vuoto.
        agent = SAC.load(args.parti_da, env=env, device="cpu")
        agent.set_random_seed(args.seed)
        agent.learning_starts = 3000
    if not args.resume:
        (args.output / "config.json").write_text(json.dumps({"algorithm": "SAC", "seed": args.seed,
            "initial_state": "upright with small random perturbations", "control_hz": 50,
            "initialized_from": str(args.parti_da) if args.parti_da else None,
            "max_episode_seconds": env.unwrapped.max_steps*env.unwrapped.dt,
            "environment": contract(disturbi)}, indent=2))
    if not args.resume and not args.parti_da:
        # Due strati da 128 ReLU: stessa taglia di base del progetto precedente,
        # ma sei ingressi. SAC usa due critici SOLO in addestramento; il firmware
        # eseguirà l'attore deterministico (media seguita da tanh).
        # Una ottimizzazione ogni quattro transizioni limita il costo CPU.
        agent = SAC("MlpPolicy", env, seed=args.seed, device="cpu", verbose=0,
                    learning_rate=3e-4, buffer_size=300000, learning_starts=3000,
                    batch_size=128, gamma=0.99, tau=0.005, train_freq=4,
                    gradient_steps=1, ent_coef="auto_0.1", policy_kwargs={"net_arch": [128, 128]})
    if disturbi is not None:
        print(f'Disturbi attivi: intensita {disturbi.intensita:g}; selezione su nominale e disturbato.', flush=True)
    print("Addestramento SAC: tutti gli episodi partono dall’alto. Ctrl+C salva ed esce.", flush=True)
    # Salviamo anche il replay buffer per poter riprendere senza perdere esperienze.
    try:
        agent.learn(total_timesteps=args.steps, callback=Progress(args.output, args.eval_every, disturbi),
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
