"""Porta una rete addestrata nel progetto STM32 (default ../Controllo_SwingUp).

Un solo comando, sempre dalla configurazione SALVATA col modello:
  1. Core/Inc/config_controllo.h: motore, rampa, scale degli ingressi, limite
     di corsa, stimatore, swing-up, controlli dell'encoder;
  2. X-CUBE-AI/App/network_data_params.c: pesi della rete (stessa forma
     6-128-128-1 del codice X-CUBE-AI gia generato);
  3. export_<cartella>/pendolo_actor.onnx con la verifica PyTorch/ONNX.

    python esporta_firmware.py --model modelli_swingup/migliore.zip
    python esporta_firmware.py --model modelli_swingup/migliore.zip --progetto ../Controllo_SwingUp
    python esporta_firmware.py --model modelli_swingup/migliore.zip --stimatore-offset -1 \
        --candidato modelli_60s_1200k/migliore.zip --candidato modelli_swingup/migliore.zip
        (piu reti nello stesso firmware, scelte al banco con rete_scelta)

Dopo: in STM32CubeIDE Project > Build Project e caricare sulla scheda.
"""
import argparse
from datetime import date
import hashlib
import math
from pathlib import Path

from addestra import check_contract
from configurazione import configurazione_modello

ROOT = Path(__file__).resolve().parent


def _f(valore):
    """Costante float C con abbastanza cifre per un float32."""
    testo = repr(float(valore))
    if 'e' not in testo and '.' not in testo:
        testo += '.0'
    testo += 'f'
    return f'({testo})' if testo.startswith('-') else testo


def header(cfg, modello):
    st, sp, ep, fw = cfg.stimatore, cfg.stepper, cfg.episodio, cfg.firmware
    import mujoco
    xml = mujoco.MjModel.from_xml_path(str(ROOT / cfg.modello.file_xml))
    finecorsa = float(xml.joint('scorrimento').range[1])
    limite = ep.limite_corsa_m or finecorsa
    cpr = cfg.disturbi.encoder_conteggi_giro if cfg.disturbi is not None else 2400
    if not math.isclose(xml.opt.timestep, 0.002):
        raise SystemExit('Il firmware esegue rampa e stimatore ogni 2 ms: serve timestep 0.002 nell XML.')
    periodo_ms = ep.periodo_controllo_s * 1000.
    if abs(periodo_ms - round(periodo_ms)) > 1e-9 or round(periodo_ms) % 2:
        raise SystemExit('periodo_controllo_s deve essere un numero pari di millisecondi.')
    periodo_ms = int(round(periodo_ms))
    salto = math.ceil(fw.omega_max_plausibile_rad_s * ep.periodo_controllo_s / (2*math.pi/cpr))
    sha = hashlib.sha256(Path(modello).read_bytes()).hexdigest()
    try:
        nome = Path(modello).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        nome = Path(modello).resolve().as_posix()
    righe = [
        '/* GENERATO da cart_pendolo_velocita_rl/esporta_firmware.py: NON modificare a mano.',
        f' * Rete: {nome} ({date.today().isoformat()})',
        f' * sha256 {sha}',
        ' * Per cambiare un valore: config.toml -> nuovo addestramento -> esporta_firmware.py.',
        ' */',
        '#ifndef CONFIG_CONTROLLO_H', '#define CONFIG_CONTROLLO_H', '',
        f'#define CFG_SWING_UP              {int(ep.swing_up)}',
        f'#define CFG_ANGOLO_INIZIALE_RAD   {_f(math.radians(ep.angolo_iniziale_gradi))}', '',
        '/* Periodo della rete ([episodio].periodo_controllo_s): letture, stimatore, inferenza. */',
        f'#define CFG_PERIODO_CONTROLLO_MS  {periodo_ms}U', '',
        '/* Motore: stessi valori di [stepper] (stepper.py). */',
        f'#define CFG_MICROSTEP             {sp.microstepping}',
        f'#define CFG_FREQ_MIN_HZ           {_f(sp.frequenza_min_hz)}',
        f'#define CFG_FREQ_MAX_HZ           {_f(sp.frequenza_max_hz)}',
        f'#define CFG_RAMPA_HZ_S            {_f(sp.rampa_hz_s)}',
        f'#define CFG_ZONA_MORTA            {_f(sp.zona_morta)}',
        f'#define CFG_METRI_PER_STEP        {_f(sp.metri_per_impulso)}',
        '/* Inversione: periodi di rampa (2 ms) con frequenza ferma a zero (stepper.py). */',
        f'#define CFG_TICK_FERMO_INVERSIONE {sp.tick_fermo_inversione}U', '',
        '/* Scale dei sei ingressi della rete (come l ambiente Python). */',
        f'#define CFG_POSIZIONE_SCALA_M     {_f(finecorsa)}',
        f'#define CFG_VELOCITA_SCALA_M_S    {_f(sp.velocita_max_m_s)}',
        f'#define CFG_OMEGA_SCALA_RAD_S     {_f(ep.scala_omega_rad_s)}', '',
        '/* Arresto del controllo a |x| >= limite (dal centro): lo stesso dell addestramento. */',
        f'#define CFG_LIMITE_CORSA_M        {_f(limite)}', '',
        '/* Encoder e controlli di sicurezza ([firmware]). */',
        f'#define CFG_ENCODER_CPR           {cpr}',
        f'#define CFG_RICALIBRA_THETA       {int(fw.ricalibra_theta_all_avvio)}',
        f'#define CFG_ANGOLO_RICALIBRAZIONE_RAD {_f(math.radians(fw.angolo_ricalibrazione_gradi))}',
        f'#define CFG_TEMPO_QUIETE_MS       {int(fw.tempo_quiete_ms)}U',
        f'#define CFG_ANGOLO_MAX_AVVIO_RAD  {_f(math.radians(fw.angolo_max_avvio_gradi))}',
        f'#define CFG_ANGOLO_MAX_CONTROLLO_RAD {_f(math.radians(fw.angolo_max_controllo_gradi))}',
        f'#define CFG_SALTO_ENCODER_MAX     {salto}U  /* conteggi in {periodo_ms} ms = {fw.omega_max_plausibile_rad_s:g} rad/s */', '',
        '/* Stimatore (stima.c), [stimatore]. */',
        f'#define STIMA_K_G      {_f(st.k_g)}',
        f'#define STIMA_K_A      {_f(st.k_a)}',
        f'#define STIMA_K_D      {_f(st.k_d)}',
        f'#define STIMA_L_THETA  {_f(st.l_theta)}',
        f'#define STIMA_L_OMEGA  {_f(st.l_omega)}',
        f'#define STIMA_L_X      {_f(st.l_x)}',
        '/* Offset dell encoder stimato (0 = stimatore a due stati). */',
        f'#define STIMA_L_B      {_f(st.l_b)}',
        f'#define STIMA_B_MAX_RAD {_f(st.b_max_rad)}', '',
        '#endif', '']
    return '\n'.join(righe)


def configurazione_esportata(modello, polo_offset=None):
    """Configurazione salvata con la rete, con lo stimatore dell offset aggiunto se richiesto."""
    check_contract(Path(modello).parent)
    cfg = configurazione_modello(Path(modello).parent)
    if polo_offset is not None:
        from dataclasses import replace
        import stima
        if cfg.stimatore.l_b:
            raise SystemExit(f'{modello}: la rete ha gia lo stimatore dell offset.')
        g = stima.guadagni_con_offset(cfg.stimatore, cfg.episodio.periodo_controllo_s, polo_offset,
                                      (cfg.stimatore.l_theta, cfg.stimatore.l_omega,
                                       cfg.episodio.periodo_controllo_s))
        cfg = replace(cfg, stimatore=replace(cfg.stimatore, **g))
        print(f'{modello}: stimatore con offset ' + ', '.join(f'{k} {v:.6f}' for k, v in g.items()))
    return cfg


# #define che possono cambiare da una rete all altra: guadagni dello stimatore
# (tabella in reti_candidate.c) e tick dell inversione (vale il massimo).
PER_RETE = ('CONFIG_CONTROLLO_H', 'CFG_TICK_FERMO_INVERSIONE',
            'STIMA_L_THETA', 'STIMA_L_OMEGA', 'STIMA_L_B', 'STIMA_L_X')


def _differenze(testo_a, testo_b, tolleranza=1e-5):
    """#define diverse tra due header (numeri confrontati con tolleranza relativa;
    il tick dell inversione e escluso, vale il massimo)."""
    import re

    def valori(testo):
        out = {}
        for r in testo.splitlines():
            m = re.match(r'#define (\w+)\s+(.*?)\s*(/\*.*)?$', r)
            if m and m[1] not in PER_RETE:
                out[m[1]] = m[2]
        return out

    def numero(v):
        try:
            return float(v.strip('()').rstrip('fU'))
        except ValueError:
            return None
    a, b = valori(testo_a), valori(testo_b)
    diverse = []
    for nome in sorted(set(a) | set(b)):
        va, vb = a.get(nome), b.get(nome)
        na, nb = numero(va or ''), numero(vb or '')
        uguali = (va == vb) or (na is not None and nb is not None
                                and abs(na - nb) <= tolleranza*max(abs(na), abs(nb), 1e-12))
        if not uguali:
            diverse.append(f'{nome}: {va} (rete 0) / {vb}')
    return diverse


def scrivi_candidati(progetto, nomi, pesi_extra, stimatori):
    """Core/Inc/reti_candidate.h e Core/Src/reti_candidate.c: la rete 0 e quella di
    network_data_params.c, le altre hanno un array di pesi con lo stesso formato."""
    from aggiorna_pesi_xcubeai import NOME_ARRAY, scrivi_array
    n = len(nomi)
    (progetto / 'Core/Inc/reti_candidate.h').write_text(
        '/* GENERATO da cart_pendolo_velocita_rl/esporta_firmware.py: NON modificare a mano. */\n'
        '#ifndef RETI_CANDIDATE_H\n#define RETI_CANDIDATE_H\n\n#include "ai_platform.h"\n\n'
        f'#define RETI_CANDIDATE_N {n}U\n\n'
        'typedef struct { float l_theta, l_omega, l_b, l_x; } GuadagniStima;\n\n'
        '/* Pesi (formato X-CUBE-AI di network_data_params.c), guadagni dello stimatore\n'
        ' * e nome di ogni rete. */\n'
        'extern const ai_handle reti_candidate_pesi[RETI_CANDIDATE_N];\n'
        'extern const GuadagniStima reti_candidate_stima[RETI_CANDIDATE_N];\n'
        'extern const char *const reti_candidate_nomi[RETI_CANDIDATE_N];\n\n#endif\n')
    righe = ['/* GENERATO da cart_pendolo_velocita_rl/esporta_firmware.py: NON modificare a mano.',
             ' * Reti selezionabili con rete_scelta (main.c), tutte con config_controllo.h:'] + \
            [f' *   {k}: {nome}' for k, nome in enumerate(nomi)] + \
            [' */', '#include "reti_candidate.h"', '#include "network_data.h"', '']
    for k, pesi in enumerate(pesi_extra, start=1):
        parole = (len(pesi) + 1)//2          # float32 + riempimento, a coppie in u64
        vuoto = f'const ai_u64 {NOME_ARRAY}[{parole}] = {{\n' + '  0x0U,\n'*parole + '};\n'
        dati = scrivi_array(vuoto, pesi)
        righe.append(dati.replace(f'const ai_u64 {NOME_ARRAY}', f'static const ai_u64 rete_candidata_{k}'))
    righe.append('const ai_handle reti_candidate_pesi[RETI_CANDIDATE_N] = {')
    righe.append(f'  AI_HANDLE_PTR({NOME_ARRAY}),')
    righe += [f'  AI_HANDLE_PTR(rete_candidata_{k}),' for k in range(1, n)]
    righe += ['};', '', 'const GuadagniStima reti_candidate_stima[RETI_CANDIDATE_N] = {']
    righe += [f'  {{ {_f(st.l_theta)}, {_f(st.l_omega)}, {_f(st.l_b)}, {_f(st.l_x)} }},' for st in stimatori]
    righe += ['};', '', 'const char *const reti_candidate_nomi[RETI_CANDIDATE_N] = {']
    righe += [f'  "{nome}",' for nome in nomi]
    righe += ['};', '']
    (progetto / 'Core/Src/reti_candidate.c').write_text('\n'.join(righe))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', type=Path, default=ROOT / 'modelli_swingup/migliore.zip',
                        help='Rete 0 (quella caricata all accensione)')
    parser.add_argument('--progetto', type=Path,
                        help='Cartella del progetto STM32 (default ../<[firmware].nome_progetto>)')
    parser.add_argument('--senza-onnx', action='store_true', help='Salta l esportazione ONNX')
    parser.add_argument('--stimatore-offset', type=float, metavar='POLO_1_S',
                        help='Per reti addestrate senza: firmware con lo stimatore dell offset '
                             '(polo in 1/s, es. -1). Guadagni da stima.guadagni_con_offset.')
    parser.add_argument('--candidato', action='append', default=[], metavar='RETE.zip[@POLO]',
                        help='Altre reti nello stesso firmware (rete 1, 2, ...), scelte con '
                             'rete_scelta dal debugger. @POLO: stimatore dell offset aggiunto. '
                             'Tutte devono avere la stessa configurazione del firmware.')
    args = parser.parse_args()
    cfg = configurazione_esportata(args.model, args.stimatore_offset)
    candidati = []
    for voce in args.candidato:
        percorso, _, polo = voce.partition('@')
        candidati.append((Path(percorso), configurazione_esportata(percorso, float(polo) if polo else None)))
    tick = max([cfg.stepper.tick_fermo_inversione] + [c.stepper.tick_fermo_inversione for _, c in candidati])
    if tick != cfg.stepper.tick_fermo_inversione:
        from dataclasses import replace
        cfg = replace(cfg, stepper=replace(cfg.stepper, tick_fermo_inversione=tick))
        print(f'Inversione con {tick} periodo fermo per tutte le reti (in simulazione differenza trascurabile).')
    progetto = args.progetto or ROOT.parent / cfg.firmware.nome_progetto
    if not (progetto / 'Core/Src/main.c').exists():
        raise SystemExit(f'{progetto} non sembra un progetto STM32 (manca Core/Src/main.c).')
    destinazione = progetto / 'Core/Inc/config_controllo.h'
    testo = header(cfg, args.model)
    for percorso, c in candidati:
        diverse = _differenze(testo, header(c, percorso))
        if diverse:
            raise SystemExit(f'{percorso}: configurazione del firmware diversa dalla rete 0:\n  '
                             + '\n  '.join(diverse))
    if args.stimatore_offset is not None:
        testo = testo.replace(' * Per cambiare un valore:',
                              f' * Stimatore dell offset aggiunto all esportazione (polo {args.stimatore_offset:g}/s):\n'
                              ' * la rete e stata addestrata con lo stimatore a due stati.\n'
                              ' * Per cambiare un valore:', 1)
    if candidati:
        testo = testo.replace(' * Per cambiare un valore:',
                              ' * Altre reti nello stesso firmware: Core/Src/reti_candidate.c.\n'
                              ' * Per cambiare un valore:', 1)
    destinazione.write_text(testo)
    print(f'Scritto {destinazione}')
    import aggiorna_pesi_xcubeai
    aggiorna_pesi_xcubeai.main(['--model', str(args.model),
                                '--params', str(progetto / 'X-CUBE-AI/App/network_data_params.c')])

    def nome(percorso, polo):
        try:
            testo_nome = Path(percorso).resolve().relative_to(ROOT).as_posix()
        except ValueError:
            testo_nome = Path(percorso).name
        return testo_nome + (' + stimatore offset' if polo else '')
    nomi = [nome(args.model, args.stimatore_offset is not None)]
    nomi += [nome(p, '@' in v) for (p, _), v in zip(candidati, args.candidato)]
    scrivi_candidati(progetto, nomi, [aggiorna_pesi_xcubeai.pesi_rete(p) for p, _ in candidati],
                     [cfg.stimatore] + [c.stimatore for _, c in candidati])
    for k, n in enumerate(nomi):
        print(f'  rete {k}: {n}')
    if not args.senza_onnx:
        import esporta_onnx
        for percorso in [args.model] + [p for p, _ in candidati]:
            esporta_onnx.main(['--model', str(percorso),
                               '--output', str(ROOT / f'export_{Path(percorso).parent.name}')])
    print('Fatto. In STM32CubeIDE: Project > Build Project, poi Run.')


if __name__ == '__main__':
    main()
