"""Addestra un agente SAC da zero o riprende un addestramento salvato.

Tutti i parametri (swing-up, ricompensa, disturbi, sensori, rete, SAC) sono in
config.toml; le opzioni da riga di comando servono solo a sovrascriverne pochi:

    python addestra.py                        # usa config.toml
    python addestra.py --config altro.toml --output modelli_prova --steps 200000
    python addestra.py --resume --output modelli_swingup --steps 300000
"""
import hashlib
from dataclasses import asdict, replace
import argparse
import csv
import json
from pathlib import Path
import time
import warnings
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from ambiente import ROOT
from configurazione import CONFIG_DEFAULT, Configurazione, configurazione_modello
from disturbi import ParametriDisturbi, crea_ambiente
from stepper import ParametriStepper

CODICE = ('ambiente.py', 'stepper.py', 'disturbi.py', 'stima.py', 'configurazione.py')


def _sha(name):
    return hashlib.sha256((ROOT / name).read_bytes()).hexdigest()


def contract(disturbi=None):
    """Firma storica (checkpoint fino al 9 ottobre): modello, codice e parametri."""
    names = ('cart_pendolo.xml', 'ambiente.py', 'stepper.py', 'stepper.toml')
    result = {"sha256": {name: _sha(name) for name in names},
              "stepper": asdict(ParametriStepper.carica()), "observation_size": 6,
              "action": "signed_normalized_step_frequency", "control_hz": 50}
    if disturbi is not None:
        for name in ('disturbi.py', 'stima.py', 'stimatore.toml'):
            result['sha256'][name] = _sha(name)
        result['disturbi'] = json.loads(json.dumps(asdict(disturbi)))
    return result


def firma(cfg):
    """Firma dei nuovi checkpoint: configurazione completa + impronte di XML e codice."""
    return {'observation_size': 6, 'action': 'signed_normalized_step_frequency',
            'control_hz': 50, 'xml_sha256': _sha(cfg.modello.file_xml),
            'codice_sha256': {name: _sha(name) for name in CODICE}}


def profilo_salvato(folder):
    return configurazione_modello(folder).disturbi


def check_contract(folder, rigoroso=False):
    """Controlla che il checkpoint sia compatibile con i file attuali.

    Errore se cambiano XML o parametri fisici. Se cambia solo il codice Python
    e un avviso (con rigoroso=True, cioe per riprendere un addestramento, errore).
    """
    folder = Path(folder)
    path = folder / 'config.json'
    if not path.exists():
        raise ValueError('Checkpoint senza config.json: addestrare in una nuova cartella.')
    saved = json.loads(path.read_text())
    if 'configurazione' in saved:
        cfg = configurazione_modello(folder)
        attuale, salvata = firma(cfg), saved.get('firma', {})
        if salvata.get('xml_sha256') != attuale['xml_sha256']:
            raise ValueError(f'{cfg.modello.file_xml} e cambiato dopo l addestramento: '
                             'addestrare in una nuova cartella.')
        diversi = [n for n in CODICE if salvata.get('codice_sha256', {}).get(n) != attuale['codice_sha256'][n]]
    else:
        attuale = contract(profilo_salvato(folder))
        salvata = saved.get('environment', {})
        fisici = [n for n in attuale['sha256'] if not n.endswith('.py')]
        if ({k: v for k, v in salvata.items() if k != 'sha256'}
                != {k: v for k, v in attuale.items() if k != 'sha256'}
                or any(salvata.get('sha256', {}).get(n) != attuale['sha256'][n] for n in fisici)):
            raise ValueError('Checkpoint incompatibile con ambiente/parametri. Addestrare in una nuova cartella.')
        diversi = [n for n in attuale['sha256'] if n.endswith('.py')
                   and salvata.get('sha256', {}).get(n) != attuale['sha256'][n]]
    if diversi:
        messaggio = (f'Codice cambiato dopo l addestramento di {folder.name}: {", ".join(diversi)}. '
                     'Con i parametri di default il comportamento storico e conservato.')
        if rigoroso:
            raise ValueError(messaggio + ' Per riprendere usare una nuova cartella (--parti-da).')
        warnings.warn(messaggio, stacklevel=2)


def evaluate(agent, episodes=5, seed_start=1000, disturbi=None, configurazione=None):
    """Policy deterministica su semi fissi; la selezione non usa i semi di test."""
    if episodes <= 0:
        raise ValueError('episodes deve essere positivo.')
    env = crea_ambiente(disturbi, configurazione=configurazione)
    rows = []
    for index in range(episodes):
        obs, _ = env.reset(seed=seed_start + index)
        total = 0.
        max_angle = max_x = 0.
        t_alto = None
        for _ in range(env.max_steps):
            action, _ = agent.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            max_angle = max(max_angle, abs(info["angle_deg"]))
            max_x = max(max_x, abs(info["cart_x"]))
            if t_alto is None and abs(info['angle_deg']) < env.ep.stabile_angolo_gradi:
                t_alto = float(env.data.time)
            if terminated or truncated:
                break
        # Swing-up: equilibrio ininterrotto da tempo_max_salita_s alla fine;
        # solo equilibrio: tutto l'episodio. Il criterio dei 3 s resta diagnostico.
        rows.append({"seed": seed_start + index, "reward": total,
                     "duration_s": float(env.data.time), "terminated": terminated, **info,
                     "three_second_success": info["is_success"],
                     "is_success": env.successo(terminated), "max_angle_deg": max_angle,
                     "max_cart_x_m": max_x, "first_upright_s": t_alto})
    env.close()
    chiave = 'final_stable_seconds' if env.ep.swing_up else 'stable_seconds'
    return {"episodes": episodes, "mean_reward": sum(r["reward"] for r in rows) / episodes,
            "success_rate": sum(r["is_success"] for r in rows) / episodes,
            "worst_stable_seconds": min(r[chiave] for r in rows),
            "mean_stable_seconds": sum(r[chiave] for r in rows) / episodes,
            "stable_metric": chiave, "details": rows}


def evaluate_comparison(agent, episodes=5, seed_start=1000, disturbi=None, configurazione=None):
    result = evaluate(agent, episodes, seed_start, disturbi, configurazione)
    result['selection_score'] = result['worst_stable_seconds']
    result['selection_reward'] = result['mean_reward']
    if disturbi is not None:
        result['nominal'] = evaluate(agent, episodes, seed_start, None, configurazione)
        # Evita di selezionare una rete che migliora sotto disturbi ma peggiora
        # molto sul nominale. Le due suite hanno gli stessi stati iniziali.
        result['selection_score'] = min(result['worst_stable_seconds'], result['nominal']['worst_stable_seconds'])
        result['selection_reward'] = min(result['mean_reward'], result['nominal']['mean_reward'])
        result['disturbi'] = json.loads(json.dumps(asdict(disturbi)))
    return result


class Progress(BaseCallback):
    def __init__(self, folder, frequency, disturbi=None, configurazione=None, episodes=5):
        super().__init__()
        self.folder, self.frequency = folder, frequency
        self.disturbi, self.configurazione, self.episodes = disturbi, configurazione, episodes
        self.versione = 3 if configurazione is None else 4
        self.started = time.monotonic()
        self.best = (float("-inf"), float("-inf"))
        if (folder / "best_metrics.json").exists():
            previous = json.loads((folder / "best_metrics.json").read_text())
            if previous.get('selection_version') == self.versione:
                self.best = (previous['selection_score'], previous['selection_reward'])

    def _on_step(self):
        # Valutare costa: eseguiamo episodi completi solo ogni frequency decisioni.
        if self.n_calls % self.frequency:
            return True
        metrics = evaluate_comparison(self.model, self.episodes, disturbi=self.disturbi,
                                      configurazione=self.configurazione)
        metrics["selection_version"] = self.versione
        metrics["timesteps"] = self.num_timesteps
        metrics["elapsed_seconds_this_run"] = time.monotonic() - self.started
        history = self.folder / "progresso.csv"
        exists = history.exists()
        nominale = metrics.get('nominal', metrics)
        with history.open("a", newline="") as stream:
            writer = csv.writer(stream)
            if not exists:
                writer.writerow(["steps", "reward", "success_rate", "elapsed_s_this_run",
                                 "success_rate_nominal", "selection_score"])
            writer.writerow([self.num_timesteps, metrics["mean_reward"],
                             metrics["success_rate"], metrics["elapsed_seconds_this_run"],
                             nominale['success_rate'], metrics['selection_score']])
        self.model.save(self.folder / "ultimo")
        # A parita di durata stabile vince la ricompensa peggiore tra nominale
        # e disturbato: evita di tenere la prima rete appena sufficiente.
        candidate = (metrics["selection_score"], metrics["selection_reward"])
        if candidate > self.best:
            self.best = candidate
            self.model.save(self.folder / "migliore")
            (self.folder / "best_metrics.json").write_text(json.dumps(metrics, indent=2))
        print(f"Passi {self.num_timesteps:,} | ricompensa {metrics['mean_reward']:.1f} "
              f"| successi {metrics['success_rate']:.0%} (nominale {nominale['success_rate']:.0%}) "
              f"| peggiore {metrics['selection_score']:.1f} s "
              f"| tempo {metrics['elapsed_seconds_this_run']/60:.1f} min", flush=True)
        return True


def _ent_coef(valore):
    try:
        return float(valore)
    except ValueError:
        return valore


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=CONFIG_DEFAULT,
                        help='File di configurazione unico (default config.toml)')
    parser.add_argument("--steps", type=int, help="Nuovi passi (default [addestramento].passi)")
    parser.add_argument("--seed", type=int, help='Default [addestramento].seed')
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--output", type=Path, help='Default [addestramento].cartella')
    parser.add_argument("--eval-every", type=int, help='Default [addestramento].valuta_ogni')
    parser.add_argument('--parti-da', type=Path, help='Nuovo addestramento che parte dai pesi di una rete .zip')
    parser.add_argument('--disturbi', type=Path,
                        help='Profilo [disturbi] in un TOML separato (sostituisce quello di --config)')
    args = parser.parse_args()
    if args.resume and args.parti_da:
        parser.error("Usa --resume oppure --parti-da, non entrambi.")
    if args.resume:
        if args.output is None:
            parser.error('--resume richiede --output con la cartella da riprendere.')
        check_contract(args.output, rigoroso=True)
        cfg = configurazione_modello(args.output)
        if args.disturbi and ParametriDisturbi.carica(args.disturbi) != cfg.disturbi:
            parser.error('Profilo disturbi diverso dal checkpoint: usare una nuova cartella.')
    else:
        cfg = Configurazione.carica(args.config)
        if args.disturbi:
            cfg = replace(cfg, disturbi=ParametriDisturbi.carica(args.disturbi),
                          addestramento=replace(cfg.addestramento, usa_disturbi=True))
    a = cfg.addestramento
    if args.seed is not None:
        cfg = replace(cfg, addestramento=replace(a, seed=args.seed))
        a = cfg.addestramento
    output = args.output or ROOT / a.cartella
    steps = args.steps or a.passi
    eval_every = args.eval_every or a.valuta_ogni
    if steps <= 0 or eval_every <= 0:
        parser.error("Passi e frequenza devono essere positivi.")
    output.mkdir(parents=True, exist_ok=True)
    if not args.resume and any(output.iterdir()):
        parser.error(f"{output} contiene gia un addestramento: usa --resume oppure --output con una nuova cartella.")
    disturbi = cfg.disturbi_addestramento
    legacy = args.resume and 'configurazione' not in json.loads((output / 'config.json').read_text())
    cfg_env = None if legacy else cfg
    torch.set_num_threads(1)
    env = Monitor(crea_ambiente(disturbi, configurazione=cfg_env, addestramento=True))
    if args.resume:
        agent = SAC.load(output / "ultimo", env=env, device="cpu")
        buffer = output / "esperienze.pkl"
        if buffer.exists():
            agent.load_replay_buffer(buffer)
        else:
            agent.learning_starts = agent.num_timesteps + 2000
            print("Buffer non disponibile: raccolgo nuove esperienze prima di aggiornare la rete.", flush=True)
    elif args.parti_da:
        # Pesi e coefficiente di entropia della rete indicata; buffer vuoto.
        agent = SAC.load(args.parti_da, env=env, device="cpu")
        agent.set_random_seed(a.seed)
        agent.learning_starts = a.learning_starts
    else:
        # SAC usa due critici SOLO in addestramento; il firmware esegue l'attore
        # deterministico (media seguita da tanh).
        agent = SAC("MlpPolicy", env, seed=a.seed, device="cpu", verbose=0,
                    learning_rate=a.learning_rate, buffer_size=a.buffer,
                    learning_starts=a.learning_starts, batch_size=a.batch, gamma=a.gamma,
                    tau=a.tau, train_freq=a.train_freq, gradient_steps=a.gradient_steps,
                    ent_coef=_ent_coef(a.ent_coef), policy_kwargs={"net_arch": list(a.strati)})
    if not args.resume:
        ep = cfg.episodio
        (output / "config.json").write_text(json.dumps({
            "algorithm": "SAC", "seed": a.seed, "control_hz": 50,
            "initial_state": (f"swing-up da {ep.angolo_iniziale_gradi:g} gradi" if ep.swing_up
                              else "upright with small random perturbations"),
            "initialized_from": str(args.parti_da) if args.parti_da else None,
            "config_file": str(Path(args.config).resolve()),
            "max_episode_seconds": env.unwrapped.max_steps*env.unwrapped.dt,
            "firma": firma(cfg), "configurazione": cfg.a_dict()}, indent=2))
        (output / 'config_usata.toml').write_bytes(Path(args.config).read_bytes())
    if disturbi is not None:
        print(f'Disturbi attivi: intensita {disturbi.intensita:g}; selezione su nominale e disturbato.', flush=True)
    if cfg.sensori.attivi and disturbi is not None:
        print('Guasti sensori attivi: ' + ', '.join(f'{k}={v:g}' for k, v in asdict(cfg.sensori).items()), flush=True)
    print(("Swing-up: episodi dal basso " if cfg.episodio.swing_up and not legacy
           else "Equilibrio: episodi dall'alto ") + f"-> {output}. Ctrl+C salva ed esce.", flush=True)
    try:
        agent.learn(total_timesteps=steps,
                    callback=Progress(output, eval_every, disturbi, cfg_env, a.episodi_selezione),
                    reset_num_timesteps=not args.resume)
    except KeyboardInterrupt:
        print("Interruzione richiesta: salvo lo stato corrente.", flush=True)
    finally:
        agent.save(output / "ultimo")
        agent.save_replay_buffer(output / "esperienze.pkl")
        env.close()
    print("Salvataggio completato.", flush=True)


if __name__ == "__main__":
    main()
