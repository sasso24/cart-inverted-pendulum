"""Simulazione della rete SAC (o del pendolo in manuale), valutazione e prove.

Ogni rete gira con la configurazione salvata con il modello (config.json):
swing-up o equilibrio, motore, stimatore, disturbi.

    python simula.py                     # finestra MuJoCo con la rete consegnata
    python simula.py --manual            # finestra senza rete, comandi manuali
    python simula.py --headless          # un episodio senza finestra, riepilogo JSON
    python simula.py valuta --episodes 20
    python simula.py grafici             # traiettoria.csv e controllo_stepper.png
    python simula.py confronta --model a/migliore.zip b/migliore.zip
    python simula.py smorzamento --smorzamenti 0.01 0.001 --attrito 0.001
    python simula.py perno --smorzamento 0.004 --attrito 0.002

`python simula.py <comando> -h` per le opzioni di ciascun comando.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np

from ambiente import (CONFIG_DEFAULT, ROOT, Configurazione, ParametriDisturbi, con_offset,
                      configurazione_modello, crea_ambiente)

RETE = ROOT / 'modelli_finale' / 'migliore.zip'


def carica_rete(modello):
    """Rete SAC e configurazione salvata con essa, dopo il controllo di compatibilita."""
    import torch
    from stable_baselines3 import SAC
    from addestra import check_contract
    torch.set_num_threads(1)
    check_contract(Path(modello).parent)
    return SAC.load(modello, device='cpu'), configurazione_modello(Path(modello).parent)


def profilo(args, cfg):
    """Disturbi: --nominale, --disturbi TOML, altrimenti quelli salvati con la rete."""
    if args.nominale:
        return None
    if args.disturbi:
        return ParametriDisturbi.carica(args.disturbi)
    return cfg.disturbi


def _opzioni_profilo(parser):
    gruppo = parser.add_mutually_exclusive_group()
    gruppo.add_argument('--disturbi', type=Path,
                        help='TOML con la sezione [disturbi] (anche un config.toml)')
    gruppo.add_argument('--nominale', action='store_true', help='Senza disturbi')


# =============================================================================
#  finestra: rete (o manuale) nella finestra MuJoCo, oppure un episodio headless
# =============================================================================

def finestra(args):
    if not args.manual and not args.model.is_file():
        raise SystemExit('Modello non trovato. Addestrare prima (addestra.py) o specificare --model.')
    agent = None
    if args.manual:
        cfg = Configurazione.carica(args.config)
        disturbi = None if args.nominale or not args.disturbi else ParametriDisturbi.carica(args.disturbi)
    else:
        agent, cfg = carica_rete(args.model)
        disturbi = profilo(args, cfg)
    # Perno fisso: niente estrazione per episodio di quel valore (ambiente.py, _perno).
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


# =============================================================================
#  valuta: semi diversi da quelli della selezione del modello
# =============================================================================

def valuta(args):
    from addestra import evaluate_comparison
    if args.episodes <= 0:
        raise SystemExit("Il numero di episodi deve essere positivo.")
    agent, cfg = carica_rete(args.model)
    result = evaluate_comparison(agent, args.episodes, seed_start=2000, disturbi=profilo(args, cfg),
                                 configurazione=cfg)
    result["model"] = str(args.model.resolve())
    ep = cfg.episodio
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


# =============================================================================
#  grafici: traiettoria e controllo di velocita in Hz
# =============================================================================

def grafici(args):
    import csv
    import os
    import tempfile
    os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'stepper_hz_mpl'))
    os.environ.setdefault('XDG_CACHE_HOME', str(Path(tempfile.gettempdir()) / 'stepper_hz_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    agent, cfg = carica_rete(args.model)
    env = crea_ambiente(profilo(args, cfg), configurazione=cfg)
    obs, info = env.reset(seed=args.seed, options={'exact': True})
    rows = [[0., info['angle_deg'], info['cart_x'], 0., 0., 0., 0.]]
    for _ in range(env.max_steps):
        action, _ = agent.predict(obs, deterministic=True)
        obs, _, done, truncated, info = env.step(action)
        rows.append([float(env.data.time), info['angle_deg'], info['cart_x'],
                     info['requested_frequency_hz'], info['motor_frequency_hz'],
                     info['command_velocity_m_s'], info['cart_velocity_m_s']])
        if done or truncated:
            break
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / 'traiettoria.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'angle_deg', 'cart_x_m', 'requested_frequency_hz',
                         'motor_frequency_hz', 'command_velocity_m_s', 'cart_velocity_m_s'])
        writer.writerows(rows)
    t, angle, x, requested, hz, target, velocity = np.asarray(rows).T
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle('Pendolo con stepper — SAC velocita in Hz')
    axes[0, 0].plot(t, np.abs(angle))
    axes[0, 0].axhspan(0, env.ep.stabile_angolo_gradi, color='green', alpha=.12)
    axes[0, 0].set(title='Distanza dalla verticale', ylabel='gradi')
    axes[0, 1].plot(t, x)
    axes[0, 1].axhline(env.limite_corsa, color='red', linestyle='--')
    axes[0, 1].axhline(-env.limite_corsa, color='red', linestyle='--')
    axes[0, 1].set(title='Posizione carrello', ylabel='m')
    axes[1, 0].step(t, requested, where='post', label='Richiesta rete', alpha=.6)
    axes[1, 0].plot(t, hz, label='Comando dopo rampa e arresti')
    axes[1, 0].set(title='Frequenza STEP firmata', ylabel='Hz')
    axes[1, 0].legend()
    axes[1, 1].plot(t, target, label='Comandata')
    axes[1, 1].plot(t, velocity, label='Simulata')
    axes[1, 1].set(title='Velocita carrello', ylabel='m/s')
    axes[1, 1].legend()
    for ax in axes.flat:
        ax.set_xlabel('Tempo (s)')
        ax.grid(alpha=.2)
    fig.savefig(args.output / 'controllo_stepper.png', dpi=150)
    plt.close(fig)
    env.close()
    print(f'Salvati traiettoria.csv e controllo_stepper.png in {args.output}')


# =============================================================================
#  confronta: robustezza di piu reti sugli stessi semi e nelle stesse condizioni
# =============================================================================
# Ogni rete gira col proprio periodo, stimatore e motore (config.json salvato),
# ma le condizioni di prova sono comuni:
#   nominale         sensori ideali, nessun disturbo
#   disturbi         profilo [disturbi] della rete e guasti [sensori] del 14/10
#                    (offset +/-2 gradi, conteggi persi fino allo 0,2%)
#   sensori_16_10    come sopra con offset +/-3 gradi e conteggi persi fino allo 0,3%
#   offset_4         offset dell'encoder fisso a +4 o -4 gradi (oltre l'addestramento)
#   inversione       disturbi del 14/10 con l'inversione del motore come sul firmware
#                    (rampa ferma a zero 2 ms; le reti del 14/10 non l'hanno vista)
#   aggancio         partenza da stato casuale (angolo su tutto il giro, omega fino a
#                    4 rad/s, carrello in moto fino a 0,3 m/s), disturbi del 14/10
# Con --stimatore-offset una rete addestrata senza stima dell'offset viene provata con
# lo stimatore a tre stati, come sul firmware esportato con esporta_firmware.py --stimatore-offset.

CONDIZIONI = ('nominale', 'disturbi', 'sensori_16_10', 'offset_4', 'inversione', 'aggancio')


def configurazione_prova(cfg, condizione):
    sensori = replace(cfg.sensori, offset_max_gradi=2.0, prob_conteggio_perso_max=0.002)
    stepper = cfg.stepper
    episodio = cfg.episodio
    if condizione == 'sensori_16_10':
        sensori = replace(sensori, offset_max_gradi=3.0, prob_conteggio_perso_max=0.003)
    if condizione == 'inversione':
        stepper = replace(stepper, tick_fermo_inversione=1)
    if condizione == 'aggancio':
        episodio = replace(episodio, frazione_partenza_alto=0., frazione_partenza_casuale=1.,
                           casuale_x_m=0.25, casuale_velocita_m_s=0.3, casuale_omega_rad_s=4.)
    return replace(cfg, sensori=sensori, stepper=stepper, episodio=episodio)


def _episodio_confronto(agent, env, seed, condizione):
    obs, _ = env.reset(seed=seed)
    if condizione == 'offset_4' and hasattr(env, 'errore_encoder_rad'):
        # Offset fisso al posto di quello estratto (segno alternato sui semi).
        env.errore_encoder_rad = math.radians(4.0 if seed % 2 else -4.0)
        env._misura(iniziale=True)
        obs = env.observation()
    totale, terminato = 0., False
    inversioni, precedente = 0, 0.
    for _ in range(env.max_steps):
        azione, _ = agent.predict(obs, deterministic=True)
        obs, r, terminato, troncato, info = env.step(azione)
        totale += r
        u = float(azione[0])
        inversioni += u * precedente < 0
        precedente = u
        if terminato or troncato:
            break
    finale = env.stable_steps * env.dt
    # Aggancio: in equilibrio dagli istanti finali (8 s su 20 s, durata-12 s su episodi piu lunghi).
    richiesto = max(8., env.max_steps*env.dt - 12.)
    successo = (not terminato and finale >= richiesto - 1e-9) if condizione == 'aggancio' \
        else env.successo(terminato)
    # Criterio piu largo: su entro 20 s (o subito, nell'aggancio) e in equilibrio fino alla fine.
    su_alla_fine = not terminato and finale >= env.max_steps*env.dt - (12. if condizione == 'aggancio' else 20.) - 1e-9
    return {'seed': seed, 'successo': bool(successo), 'su_alla_fine': bool(su_alla_fine),
            'stabile_finale_s': finale,
            'terminato': bool(terminato), 'motivo': info['termination_reason'],
            'inversioni_al_s': inversioni / max(env.steps * env.dt, 1e-9), 'ricompensa': totale}


def _valuta_condizione(modello, condizione, episodi, seed_start, polo_offset=None, durata=None):
    agent, cfg = carica_rete(modello)
    cfg = configurazione_prova(con_offset(cfg, polo_offset), condizione)
    if durata is not None:
        # Stessa durata per tutte le reti (successo: in equilibrio da tempo_max_salita alla fine).
        cfg = replace(cfg, episodio=replace(cfg.episodio, durata_s=durata))
    disturbi = None if condizione == 'nominale' else cfg.disturbi
    env = crea_ambiente(disturbi, configurazione=cfg, addestramento=(condizione == 'aggancio'))
    if condizione == 'nominale':
        env.sensori = None
    righe = [_episodio_confronto(agent, env, seed_start + i, condizione) for i in range(episodi)]
    return {'successi': sum(r['successo'] for r in righe), 'episodi': episodi,
            'su_alla_fine': sum(r['su_alla_fine'] for r in righe),
            'inversioni_al_s': float(np.mean([r['inversioni_al_s'] for r in righe])),
            'periodo_s': env.dt, 'dettagli': righe}


def confronta(args):
    risultati = {}
    for modello in args.model:
        risultati[str(modello)] = {}
        for condizione in args.condizioni:
            r = _valuta_condizione(modello, condizione, args.episodes, args.seed_start,
                                   args.stimatore_offset, args.durata)
            risultati[str(modello)][condizione] = r
            print(f'{modello} | {condizione:14s} | {r["successi"]:2d}/{r["episodi"]} '
                  f'(su entro 20 s e fino alla fine {r["su_alla_fine"]:2d}) '
                  f'| inversioni {r["inversioni_al_s"]:.1f}/s', flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(risultati, indent=2))


# =============================================================================
#  smorzamento: sweep dello smorzamento del perno (la rete aggancia se frena poco?)
# =============================================================================
# Diagnosi del 24/10: con il damping 0.01 dell'XML storico le reti non passano mai
# sopra la verticale; con un perno reale che frena meno il pendolo arriva in alto
# troppo veloce, gira e non aggancia. Qui ogni episodio usa uno smorzamento del
# perno FISSO (sostituisce quello dell'XML e quello estratto da [disturbi]), con
# attrito coulombiano e resistenza dell'aria fissi (default 0). Lo stimatore resta
# quello salvato con la rete (come sul firmware).
#   nominale  stato vero, nessun disturbo ne stimatore
#   disturbi  profilo [disturbi] e [sensori] della rete, stimatore della rete
# Per ogni smorzamento: successi, passaggi sopra la verticale (entra nella meta alta
# da un lato ed esce dall'altro) e primo arrivo in alto (|angolo| < stabile_angolo_gradi).

SMORZAMENTI = (0.012, 0.01, 0.005, 0.0025, 0.001, 0.0003)


def _episodio_perno(agent, env, seed, smorzamento, attrito, aria):
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


def _prova_smorzamento(modello, condizione, smorzamento, attrito, aria, episodi, seed_start, durata):
    agent, cfg = carica_rete(modello)
    if durata is not None:
        cfg = replace(cfg, episodio=replace(cfg.episodio, durata_s=durata))
    env = crea_ambiente(None if condizione == 'nominale' else cfg.disturbi, configurazione=cfg)
    if condizione == 'nominale':
        env.sensori = None
    righe = [_episodio_perno(agent, env, seed_start + i, smorzamento, attrito, aria) for i in range(episodi)]
    arrivi = [r['primo_arrivo_s'] for r in righe if r['primo_arrivo_s'] is not None]
    return {'modello': str(modello), 'condizione': condizione, 'smorzamento_nms': smorzamento,
            'successi': sum(r['successo'] for r in righe), 'episodi': episodi,
            'passaggi_medi': float(np.mean([r['passaggi'] for r in righe])),
            'primo_arrivo_mediano_s': float(np.median(arrivi)) if arrivi else None,
            'mai_in_alto': episodi - len(arrivi), 'dettagli': righe}


def smorzamento(args):
    lavori = [(str(m), c, s, args.attrito, args.aria, args.episodes, args.seed_start, args.durata)
              for m in args.model for s in args.smorzamenti for c in args.condizioni]
    with ProcessPoolExecutor(args.processi) as pool:
        risultati = list(pool.map(_prova_smorzamento, *zip(*lavori)))
    for r in risultati:
        arrivo = r['primo_arrivo_mediano_s']
        print(f"{r['modello']} | {r['condizione']:8s} | damping {r['smorzamento_nms']:.4f} | "
              f"{r['successi']:2d}/{r['episodi']} | passaggi {r['passaggi_medi']:5.1f} | "
              f"primo arrivo {'-' if arrivo is None else f'{arrivo:.1f} s'}"
              f"{f' (mai in alto {r["mai_in_alto"]})' if r['mai_in_alto'] else ''}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'attrito_nm': args.attrito, 'aria_nms2': args.aria,
                                       'risultati': risultati}, indent=2))


# =============================================================================
#  perno: pendolo libero col carrello fermo, per confrontare il perno col banco
# =============================================================================
# Parte da un angolo (default 90 gradi) e cade; si cambiano attrito e smorzamento
# del perno dalla finestra e si guarda quanto oscilla (tempo per scendere sotto
# 45, 20, 10, 5, 2, 1 gradi e per fermarsi). Non modifica XML, configurazioni o
# reti: i valori cambiano solo in memoria.
#
# Al banco: motore fermo (driver acceso, carrello tenuto), pendolo a 90 gradi, lasciarlo
# e cronometrare. Calo dell'ampiezza costante a ogni oscillazione = attrito coulombiano;
# calo in proporzione all'ampiezza (rapporto costante) = smorzamento viscoso.

SOGLIE_GRADI = (45., 20., 10., 5., 2., 1.)
FERMO_GRADI = 0.5
OMEGA_FERMO = 0.005   # rad/s


class ProvaPerno:
    def __init__(self, xml, smorzamento, attrito, aria, angolo):
        import mujoco
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        giunto = self.model.joint('rotazione')
        self.dof, self.qadr = int(giunto.dofadr[0]), int(giunto.qposadr[0])
        self.smorzamento, self.attrito, self.aria, self.angolo = smorzamento, attrito, aria, angolo
        self.riparti()

    def applica(self):
        self.model.dof_damping[self.dof] = self.smorzamento
        self.model.dof_frictionloss[self.dof] = self.attrito

    def riparti(self):
        self.mujoco.mj_resetData(self.model, self.data)
        # Angoli dell'XML: 0 = braccio lungo in alto, pi = appeso.
        self.data.qpos[self.qadr] = math.pi + math.radians(self.angolo)
        self.data.ctrl[:] = 0.          # attuatore di velocita a zero: carrello tenuto fermo
        self.applica()
        self.mujoco.mj_forward(self.model, self.data)
        self.picchi = [(0., abs(self.angolo))]   # (t, ampiezza in gradi da appeso)
        self.soglie = {}
        self.fermo_t = self.quiete_da = None

    @property
    def angolo_da_appeso(self):
        return math.degrees(math.remainder(float(self.data.qpos[self.qadr]) - math.pi, 2*math.pi))

    def passo(self):
        omega = float(self.data.qvel[self.dof])
        self.data.qfrc_applied[self.dof] = -self.aria*omega*abs(omega)
        self.mujoco.mj_step(self.model, self.data)
        if self.fermo_t is not None:
            return
        nuova = float(self.data.qvel[self.dof])
        t = float(self.data.time)
        # Picco = inversione del verso; 0,2 s minimi tra due picchi contro il rumore numerico.
        if omega*nuova <= 0 and omega != 0 and t - self.picchi[-1][0] > 0.2:
            ampiezza = abs(self.angolo_da_appeso)
            self.picchi.append((t, ampiezza))
            for s in SOGLIE_GRADI:
                if ampiezza < s and s not in self.soglie:
                    self.soglie[s] = t
            if ampiezza < FERMO_GRADI:
                self.fermo_t = t
        # Bloccato dall'attrito coulombiano (anche a qualche decimo di grado dal fondo):
        # velocita quasi nulla per 1 s; il tempo di arresto e l'inizio di quel secondo.
        if abs(nuova) > OMEGA_FERMO:
            self.quiete_da = None
        elif self.quiete_da is None:
            self.quiete_da = t
        elif t - self.quiete_da >= 1. and self.fermo_t is None:
            self.fermo_t = self.quiete_da

    def periodo(self):
        if len(self.picchi) < 3:
            return None
        t = [p[0] for p in self.picchi[1:]]
        return 2*(t[-1] - t[0])/(len(t) - 1) if len(t) > 1 else None

    def calo(self):
        """Ultimo calo per semi-oscillazione (gradi) e rapporto tra due ampiezze successive."""
        if len(self.picchi) < 2:
            return None, None
        a0, a1 = self.picchi[-2][1], self.picchi[-1][1]
        return a0 - a1, (a1/a0 if a0 else None)

    def parametri(self):
        return (f'smorzamento {self.smorzamento:.5f} N*m*s | attrito {self.attrito:.5f} N*m | '
                f'aria {self.aria:.2e} N*m*s^2 | partenza {self.angolo:.0f} gradi')

    def soglie_testo(self):
        parti = [f'<{s:g}: ' + (f'{self.soglie[s]:.1f} s' if s in self.soglie else '-') for s in SOGLIE_GRADI]
        parti.append('ferma: ' + (f'{self.fermo_t:.1f} s' if self.fermo_t is not None else '-'))
        return ' | '.join(parti)

    def riepilogo(self):
        oscillazioni = (len(self.picchi) - 1)/2
        periodo = self.periodo()
        return (self.parametri() + '\n  ' + self.soglie_testo()
                + f'\n  oscillazioni complete {oscillazioni:.1f}'
                + (f' | periodo {periodo:.3f} s' if periodo else '') + f' | t {self.data.time:.1f} s')


def perno(args):
    if min(args.smorzamento, args.attrito, args.aria) < 0:
        raise SystemExit('Valori non negativi.')
    prova = ProvaPerno(args.xml, args.smorzamento, args.attrito, args.aria, args.angolo)
    if not args.headless:
        from finestra import run_perno
        run_perno(prova)
        return
    while prova.data.time < args.durata and prova.fermo_t is None:
        prova.passo()
    print(prova.parametri())
    print(' osc.     t (s)   ampiezza (gradi)   calo per osc. (gradi)   rapporto')
    picchi = prova.picchi[::2]   # una riga per oscillazione completa (stesso lato)
    for k, (t, a) in enumerate(picchi):
        if k == 0:
            print(f'{k:4d} {t:9.2f} {a:12.2f}')
        else:
            a0 = picchi[k-1][1]
            print(f'{k:4d} {t:9.2f} {a:12.2f} {a0 - a:18.2f} {a/a0 if a0 else 0:16.3f}')
    print(prova.riepilogo())


# =============================================================================

def main(argv=None):
    import sys
    argv = list(sys.argv[1:] if argv is None else argv)
    comandi = ('finestra', 'valuta', 'grafici', 'confronta', 'smorzamento', 'perno')
    if not argv or argv[0] not in comandi + ('-h', '--help'):
        argv.insert(0, 'finestra')
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sotto = parser.add_subparsers(dest='comando', required=True)

    p = sotto.add_parser('finestra', help='Finestra MuJoCo o episodio headless (default)')
    p.add_argument('--model', type=Path, default=RETE)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--manual', action='store_true', help='Senza rete addestrata; usa le frecce')
    _opzioni_profilo(p)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--config', type=Path, default=CONFIG_DEFAULT,
                   help='Solo con --manual: episodio e motore da questo file')
    p.add_argument('--smorzamento', type=float, metavar='N_M_S',
                   help='Smorzamento viscoso del perno fisso in N*m*s/rad '
                        '(default: XML, o estratto a ogni reset con i disturbi della rete)')
    p.add_argument('--attrito', type=float, metavar='N_M',
                   help='Attrito coulombiano del perno fisso in N*m (default come sopra)')
    p.set_defaults(funzione=finestra)

    p = sotto.add_parser('valuta', help='Valutazione su semi separati (2000 e successivi)')
    p.add_argument('--model', type=Path, default=RETE)
    p.add_argument('--episodes', type=int, default=20)
    p.add_argument('--output', type=Path, default=ROOT / 'risultati' / 'valutazione.json')
    _opzioni_profilo(p)
    p.set_defaults(funzione=valuta)

    p = sotto.add_parser('grafici', help='CSV e grafici di una traiettoria')
    p.add_argument('--model', type=Path, default=RETE)
    p.add_argument('--output', type=Path, default=ROOT / 'risultati')
    _opzioni_profilo(p)
    p.add_argument('--seed', type=int, default=42)
    p.set_defaults(funzione=grafici)

    p = sotto.add_parser('confronta', help='Piu reti sugli stessi semi in piu condizioni')
    p.add_argument('--model', type=Path, nargs='+', default=[RETE])
    p.add_argument('--condizioni', nargs='+', default=list(CONDIZIONI), choices=CONDIZIONI)
    p.add_argument('--episodes', type=int, default=40)
    p.add_argument('--seed-start', type=int, default=2000)
    p.add_argument('--output', type=Path, default=ROOT / 'risultati' / 'confronto.json')
    p.add_argument('--durata', type=float, help='Durata degli episodi in s (default quella della rete)')
    p.add_argument('--stimatore-offset', type=float, metavar='POLO_1_S',
                   help='Reti senza stima dell offset: provale con lo stimatore a tre stati')
    p.set_defaults(funzione=confronta)

    p = sotto.add_parser('smorzamento', help='Sweep dello smorzamento del perno')
    p.add_argument('--model', type=Path, nargs='+', default=[RETE])
    p.add_argument('--condizioni', nargs='+', default=['nominale', 'disturbi'],
                   choices=('nominale', 'disturbi'))
    p.add_argument('--smorzamenti', type=float, nargs='+', default=list(SMORZAMENTI),
                   help='Smorzamento viscoso del perno in N*m*s/rad')
    p.add_argument('--attrito', type=float, default=0., help='Attrito coulombiano del perno in N*m')
    p.add_argument('--aria', type=float, default=0., help='Resistenza dell aria in N*m*s^2 (solo disturbi)')
    p.add_argument('--episodes', type=int, default=20)
    p.add_argument('--seed-start', type=int, default=3000)
    p.add_argument('--durata', type=float, default=20., help='Durata in s (default 20)')
    p.add_argument('--processi', type=int, default=4)
    p.add_argument('--output', type=Path, default=ROOT / 'risultati' / 'prova_smorzamento.json')
    p.set_defaults(funzione=smorzamento)

    p = sotto.add_parser('perno', help='Pendolo libero: attrito e smorzamento del perno',
                         description='Tasti: Q/A smorzamento x1,25 / :1,25   W/S attrito   E/D aria   '
                                     'Z tutto a zero   frecce su/giu angolo +/-10 gradi   R riparti   '
                                     'spazio pausa   Esc esci')
    p.add_argument('--xml', type=Path, default=ROOT / 'cart_pendolo_asta20.xml')
    p.add_argument('--smorzamento', type=float, default=0.002, help='Viscoso, N*m*s/rad')
    p.add_argument('--attrito', type=float, default=0.001, help='Coulombiano, N*m')
    p.add_argument('--aria', type=float, default=0., help='Coppia -c*omega*|omega|, c in N*m*s^2')
    p.add_argument('--angolo', type=float, default=90., help='Partenza in gradi da appeso (90 = orizzontale)')
    p.add_argument('--headless', action='store_true', help='Solo tabella nel terminale')
    p.add_argument('--durata', type=float, default=300., help='Con --headless: tempo massimo in s')
    p.set_defaults(funzione=perno)

    args = parser.parse_args(argv)
    args.funzione(args)


if __name__ == '__main__':
    main()
