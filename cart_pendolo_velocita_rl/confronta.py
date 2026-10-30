"""Confronto di robustezza tra reti, sugli stessi semi e nelle stesse condizioni.

Ogni rete gira col proprio periodo, stimatore e motore (config.json salvato),
ma le condizioni di prova sono comuni:
  nominale         sensori ideali, nessun disturbo
  disturbi         profilo [disturbi] della rete e guasti [sensori] del 14/10
                   (offset +/-2 gradi, conteggi persi fino allo 0,2%)
  sensori_16_10    come sopra con offset +/-3 gradi e conteggi persi fino allo 0,3%
  offset_4         offset dell'encoder fisso a +4 o -4 gradi (oltre l'addestramento)
  inversione       disturbi del 14/10 con l'inversione del motore come sul firmware
                   (rampa ferma a zero 2 ms; le reti del 14/10 non l'hanno vista)
  aggancio         partenza da stato casuale (angolo su tutto il giro, omega fino a
                   4 rad/s, carrello in moto fino a 0,3 m/s), disturbi del 14/10

    python confronta.py --model modelli_asta20_basevecchia_storia1s_20s/migliore.zip altra_rete/migliore.zip
    python confronta.py --model modelli_asta20_basevecchia_storia1s_20s/migliore.zip --stimatore-offset -1
        (rete addestrata senza, provata con lo stimatore dell'offset come sul firmware
         esportato con esporta_firmware.py --stimatore-offset -1)
"""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import SAC

from addestra import check_contract
from configurazione import configurazione_modello
from disturbi import crea_ambiente

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


def episodio(agent, env, seed, condizione):
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


def con_offset(cfg, polo):
    import stima
    if polo is None or cfg.stimatore.l_b:
        return cfg
    st, periodo = cfg.stimatore, cfg.episodio.periodo_controllo_s
    g = stima.guadagni_con_offset(st, periodo, polo, (st.l_theta, st.l_omega, periodo))
    return replace(cfg, stimatore=replace(st, **g))


def valuta(modello, condizione, episodi, seed_start, polo_offset=None, durata=None):
    check_contract(Path(modello).parent)
    cfg = con_offset(configurazione_modello(Path(modello).parent), polo_offset)
    cfg = configurazione_prova(cfg, condizione)
    if durata is not None:
        # Stessa durata per tutte le reti (successo: in equilibrio da tempo_max_salita alla fine).
        cfg = replace(cfg, episodio=replace(cfg.episodio, durata_s=durata))
    agent = SAC.load(modello, device='cpu')
    disturbi = None if condizione == 'nominale' else cfg.disturbi
    env = crea_ambiente(disturbi, configurazione=cfg, addestramento=(condizione == 'aggancio'))
    if condizione == 'nominale':
        env.sensori = None
    righe = [episodio(agent, env, seed_start + i, condizione) for i in range(episodi)]
    return {'successi': sum(r['successo'] for r in righe), 'episodi': episodi,
            'su_alla_fine': sum(r['su_alla_fine'] for r in righe),
            'inversioni_al_s': float(np.mean([r['inversioni_al_s'] for r in righe])),
            'periodo_s': env.dt, 'dettagli': righe}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', type=Path, nargs='+', required=True)
    parser.add_argument('--condizioni', nargs='+', default=list(CONDIZIONI), choices=CONDIZIONI)
    parser.add_argument('--episodes', type=int, default=40)
    parser.add_argument('--seed-start', type=int, default=2000)
    parser.add_argument('--output', type=Path, default=Path('risultati/confronto.json'))
    parser.add_argument('--durata', type=float, help='Durata degli episodi in s (default quella della rete)')
    parser.add_argument('--stimatore-offset', type=float, metavar='POLO_1_S',
                        help='Reti senza stima dell offset: provale con lo stimatore a tre stati')
    args = parser.parse_args()
    torch.set_num_threads(1)
    risultati = {}
    for modello in args.model:
        risultati[str(modello)] = {}
        for condizione in args.condizioni:
            r = valuta(modello, condizione, args.episodes, args.seed_start, args.stimatore_offset,
                       args.durata)
            risultati[str(modello)][condizione] = r
            print(f'{modello} | {condizione:14s} | {r["successi"]:2d}/{r["episodi"]} '
                  f'(su entro 20 s e fino alla fine {r["su_alla_fine"]:2d}) '
                  f'| inversioni {r["inversioni_al_s"]:.1f}/s', flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(risultati, indent=2))


if __name__ == '__main__':
    main()
