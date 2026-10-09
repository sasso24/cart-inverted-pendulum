"""Hardware-in-the-loop: simulazione MuJoCo sul PC, stimatore e rete sulla STM32.

La scheda deve avere il firmware compilato con HIL_MODE 1 (vedi HIL.md).
A ogni passo da 20 ms il PC manda alla scheda le misure simulate (rumorose,
con i disturbi di disturbi.toml) e le velocita' applicate dalla rampa; la
scheda esegue stima.c e la rete X-CUBE-AI e restituisce il comando, che
pilota la simulazione. In parallelo il PC calcola stima.py e la rete PyTorch
per confrontare i due risultati passo per passo.

    python hil.py                      # porta trovata da sola, 5 episodi da 20 s
    python hil.py --porta /dev/cu.usbmodem1103 --episodi 10 --secondi 60
    python hil.py --ping               # solo verifica della connessione
"""
import argparse
import csv
import json
import math
from pathlib import Path
import struct
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parent

SYNC = b'\xa5\x5a'
FORMATO_RICHIESTA = '<BBI10ffff'          # 58 byte
FORMATO_RISPOSTA = '<IIff6f3f2I'          # 60 byte
DIM_RISPOSTA = struct.calcsize(FORMATO_RISPOSTA)
assert struct.calcsize(FORMATO_RICHIESTA) == 58 and DIM_RISPOSTA == 60
ERRORI = {0: 'ok', 2: 'rete non pronta o inferenza fallita', 3: 'uscita non finita',
          8: 'richiesta non valida'}


def checksum(dati):
    return (-sum(dati)) & 0xFF


class Scheda:
    def __init__(self, porta, baud=115200, timeout=0.5):
        import serial
        self.ser = serial.Serial(porta, baud, timeout=timeout)
        self.seq = int(time.time()) & 0xFFFF   # diverso a ogni avvio
        self.ritrasmissioni = 0
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def _scambia(self, comando, v=(), x=0., theta=0., applied_hz=0.):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        v = list(v)[:10]
        corpo = struct.pack(FORMATO_RICHIESTA, comando, len(v), self.seq,
                            *(v + [0.]*(10-len(v))), x, theta, applied_hz)
        pacchetto = SYNC + corpo + bytes([checksum(corpo)])
        for tentativo in range(4):
            self.ser.write(pacchetto)
            risposta = self._leggi()
            if risposta is not None and risposta[0] == self.seq:
                return risposta
            self.ritrasmissioni += 1
            self.ser.reset_input_buffer()
        raise TimeoutError('La scheda non risponde: firmware HIL caricato? porta giusta?')

    def _leggi(self):
        fine = time.monotonic() + self.ser.timeout
        precedente = b''
        while time.monotonic() < fine:
            b = self.ser.read(1)
            if not b:
                continue
            if precedente == b'\xa5' and b == b'\x5a':
                corpo = self.ser.read(DIM_RISPOSTA + 1)
                if len(corpo) != DIM_RISPOSTA + 1 or checksum(corpo[:-1]) != corpo[-1]:
                    return None
                return struct.unpack(FORMATO_RISPOSTA, corpo[:-1])
            precedente = b
        return None

    def ping(self):
        return self._scambia(2)

    def inizio(self, x, theta):
        return self._scambia(0, (), x, theta, 0.)

    def passo(self, velocita, x, theta, applied_hz):
        return self._scambia(1, velocita, x, theta, applied_hz)

    def close(self):
        self.ser.close()


def trova_porta():
    from serial.tools import list_ports
    porte = list(list_ports.comports())
    for p in porte:
        testo = f'{p.device} {p.description} {p.manufacturer}'.lower()
        if 'stlink' in testo or 'st-link' in testo or 'stmicro' in testo:
            return p.device
    for p in porte:
        if 'usbmodem' in p.device or 'ttyACM' in p.device:
            return p.device
    raise SystemExit('Nessuna scheda trovata: collegala via USB o usa --porta. '
                     f'Porte viste: {[p.device for p in porte]}')


def decodifica(r):
    seq, errore, azione, hz, *resto = r
    obs, (sx, sth, som, rete_us, tot_us) = np.array(resto[:6]), resto[6:]
    return dict(errore=errore, azione=azione, hz=hz, obs=obs, stima=(sx, sth, som),
                rete_us=rete_us, totale_us=tot_us)


def episodio(scheda, env, policy, seed, passi, scrittore, indice):
    registrate = []
    originale = env.stimatore.predici

    def registra(v, dt):
        registrate.append(v)
        originale(v, dt)
    env.stimatore.predici = registra

    _, info = env.reset(seed=seed)
    x_m, th_m = env.misura_grezza
    r = decodifica(scheda.inizio(x_m, th_m))
    stat = dict(seed=seed, passi=0, terminato=False, angolo_max_deg=0., x_max_m=0.,
                diff_obs_max=0., diff_azione_max=0., rete_us_max=0, totale_us_max=0,
                rete_us_medio=0.)
    tempi = []
    for k in range(passi):
        if r['errore']:
            raise RuntimeError(f"Errore della scheda {r['errore']}: {ERRORI.get(r['errore'])}")
        obs_pc = env.observation()
        azione_pc_su_obs_scheda = float(policy(r['obs']))
        azione_pc = float(policy(obs_pc))
        diff_obs = float(np.max(np.abs(r['obs'] - obs_pc)))
        diff_az = abs(r['azione'] - azione_pc_su_obs_scheda)
        stat['diff_obs_max'] = max(stat['diff_obs_max'], diff_obs)
        stat['diff_azione_max'] = max(stat['diff_azione_max'], diff_az)
        tempi.append(r['rete_us'])
        stat['rete_us_max'] = max(stat['rete_us_max'], r['rete_us'])
        stat['totale_us_max'] = max(stat['totale_us_max'], r['totale_us'])
        scrittore.writerow([indice, seed, k, round(env.data.time, 3),
                            *np.round(r['obs'], 6), *np.round(obs_pc, 6),
                            round(r['azione'], 6), round(azione_pc_su_obs_scheda, 6),
                            round(azione_pc, 6), round(r['hz'], 1),
                            *[round(v, 6) for v in r['stima']],
                            round(float(env.data.qpos[0]), 5), round(info['angle_deg'], 4),
                            r['rete_us'], r['totale_us']])
        registrate.clear()
        # Il comando della SCHEDA pilota la simulazione.
        _, _, terminato, troncato, info = env.step(np.array([r['azione']], dtype=np.float32))
        stat['passi'] = k + 1
        stat['angolo_max_deg'] = max(stat['angolo_max_deg'], abs(info['angle_deg']))
        stat['x_max_m'] = max(stat['x_max_m'], abs(info['cart_x']))
        if terminato or troncato:
            stat['terminato'] = terminato
            stat['motivo'] = info['termination_reason']
            break
        x_m, th_m = env.misura_grezza
        r = decodifica(scheda.passo(registrate, x_m, th_m, env.comando_motore.frequenza_hz))
    stat['rete_us_medio'] = float(np.mean(tempi)) if tempi else 0.
    stat['successo'] = (not stat['terminato']) and env.best_stable_steps == env.steps
    stat['secondi_stabili'] = env.best_stable_steps * env.dt
    env.stimatore.predici = originale
    return stat


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--porta', help='Es. /dev/cu.usbmodem1103 (Mac) o COM5 (Windows)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--episodi', type=int, default=5)
    parser.add_argument('--secondi', type=float, default=20.)
    parser.add_argument('--seed', type=int, default=7000)
    parser.add_argument('--model', type=Path, default=ROOT / 'modelli_disturbi/migliore.zip')
    parser.add_argument('--disturbi', type=Path, default=ROOT / 'disturbi.toml')
    parser.add_argument('--intensita', type=float, help='Sovrascrive intensita del profilo')
    parser.add_argument('--output', type=Path, default=ROOT / 'risultati/hil')
    parser.add_argument('--ping', action='store_true', help='Verifica solo la connessione')
    args = parser.parse_args()

    porta = args.porta or trova_porta()
    print(f'Porta: {porta}', flush=True)
    scheda = Scheda(porta, args.baud)
    try:
        t = time.monotonic()
        scheda.ping()
        print(f'Scheda HIL raggiunta (ping {1000*(time.monotonic()-t):.1f} ms).', flush=True)
        if args.ping:
            return

        import torch
        from dataclasses import replace
        from stable_baselines3 import SAC
        from disturbi import ParametriDisturbi, crea_ambiente
        torch.set_num_threads(1)
        agent = SAC.load(args.model, device='cpu')

        def policy(obs):
            return agent.predict(np.asarray(obs, dtype=np.float32), deterministic=True)[0][0]

        profilo = ParametriDisturbi.carica(args.disturbi)
        if args.intensita is not None:
            profilo = replace(profilo, intensita=args.intensita)
        if profilo.intensita == 0:
            raise SystemExit("Serve intensita > 0: l'HIL usa le misure dell'ambiente disturbato.")
        env = crea_ambiente(profilo)
        passi = int(round(args.secondi / env.dt))
        env.max_steps = passi

        args.output.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        percorso_csv = args.output / f'hil_{stamp}.csv'
        risultati = []
        with percorso_csv.open('w', newline='') as stream:
            w = csv.writer(stream)
            w.writerow(['episodio', 'seed', 'passo', 't_s',
                        *[f'obs{i}_scheda' for i in range(6)], *[f'obs{i}_pc' for i in range(6)],
                        'azione_scheda', 'azione_pytorch_su_obs_scheda', 'azione_pytorch_su_obs_pc',
                        'richiesta_hz_scheda', 'stima_x_scheda', 'stima_theta_scheda',
                        'stima_omega_scheda', 'x_vero_m', 'angolo_vero_deg', 'rete_us', 'totale_us'])
            for i in range(args.episodi):
                t0 = time.monotonic()
                s = episodio(scheda, env, policy, args.seed + i, passi, w, i)
                s['durata_reale_s'] = time.monotonic() - t0
                risultati.append(s)
                print(f"Episodio {i+1}/{args.episodi} seed {s['seed']}: "
                      f"{'OK' if s['successo'] else 'CADUTO'} | stabile {s['secondi_stabili']:.1f} s | "
                      f"angolo max {s['angolo_max_deg']:.1f} deg | "
                      f"diff obs {s['diff_obs_max']:.1e} | diff azione {s['diff_azione_max']:.1e} | "
                      f"rete {s['rete_us_medio']:.0f} us (max {s['rete_us_max']})", flush=True)
        riepilogo = {
            'porta': porta, 'modello': str(args.model), 'profilo': str(args.disturbi),
            'intensita': profilo.intensita, 'secondi': args.secondi, 'csv': percorso_csv.name,
            'successi': sum(r['successo'] for r in risultati), 'episodi': len(risultati),
            'diff_obs_max': max(r['diff_obs_max'] for r in risultati),
            'diff_azione_max': max(r['diff_azione_max'] for r in risultati),
            'rete_us_max': max(r['rete_us_max'] for r in risultati),
            'totale_us_max': max(r['totale_us_max'] for r in risultati),
            'ritrasmissioni': scheda.ritrasmissioni, 'dettagli': risultati,
            'note': ('diff_obs: ingressi della rete calcolati dalla scheda (stima.c, float32) '
                     'contro stima.py sul PC; diff_azione: uscita X-CUBE-AI contro PyTorch '
                     'sugli stessi ingressi della scheda.')}
        percorso_json = args.output / f'hil_{stamp}.json'
        percorso_json.write_text(json.dumps(riepilogo, indent=2))
        print(f"\nSuccessi {riepilogo['successi']}/{riepilogo['episodi']} | "
              f"diff obs max {riepilogo['diff_obs_max']:.1e} | "
              f"diff azione max {riepilogo['diff_azione_max']:.1e} | "
              f"rete max {riepilogo['rete_us_max']} us | ritrasmissioni {scheda.ritrasmissioni}")
        print(f'Log: {percorso_csv}\nRiepilogo: {percorso_json}')
    finally:
        scheda.close()


if __name__ == '__main__':
    sys.exit(main())
