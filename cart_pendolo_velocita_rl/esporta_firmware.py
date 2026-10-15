"""Porta una rete addestrata nel progetto STM32 (default ../Controllo_SwingUp).

Un solo comando, sempre dalla configurazione SALVATA col modello:
  1. Core/Inc/config_controllo.h: motore, rampa, scale degli ingressi, limite
     di corsa, stimatore, swing-up, controlli dell'encoder;
  2. X-CUBE-AI/App/network_data_params.c: pesi della rete (stessa forma
     6-128-128-1 del codice X-CUBE-AI gia generato);
  3. export_<cartella>/pendolo_actor.onnx con la verifica PyTorch/ONNX.

    python esporta_firmware.py --model modelli_swingup/migliore.zip
    python esporta_firmware.py --model modelli_swingup/migliore.zip --progetto ../Controllo_SwingUp

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
    salto = math.ceil(fw.omega_max_plausibile_rad_s * 0.02 / (2*math.pi/cpr))
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
        '/* Motore: stessi valori di [stepper] (stepper.py). */',
        f'#define CFG_MICROSTEP             {sp.microstepping}',
        f'#define CFG_FREQ_MIN_HZ           {_f(sp.frequenza_min_hz)}',
        f'#define CFG_FREQ_MAX_HZ           {_f(sp.frequenza_max_hz)}',
        f'#define CFG_RAMPA_HZ_S            {_f(sp.rampa_hz_s)}',
        f'#define CFG_ZONA_MORTA            {_f(sp.zona_morta)}',
        f'#define CFG_METRI_PER_STEP        {_f(sp.metri_per_impulso)}', '',
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
        f'#define CFG_SALTO_ENCODER_MAX     {salto}U  /* conteggi in 20 ms = {fw.omega_max_plausibile_rad_s:g} rad/s */', '',
        '/* Stimatore (stima.c), [stimatore]. */',
        f'#define STIMA_K_G      {_f(st.k_g)}',
        f'#define STIMA_K_A      {_f(st.k_a)}',
        f'#define STIMA_K_D      {_f(st.k_d)}',
        f'#define STIMA_L_THETA  {_f(st.l_theta)}',
        f'#define STIMA_L_OMEGA  {_f(st.l_omega)}',
        f'#define STIMA_L_X      {_f(st.l_x)}', '',
        '#endif', '']
    return '\n'.join(righe)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', type=Path, default=ROOT / 'modelli_swingup/migliore.zip')
    parser.add_argument('--progetto', type=Path,
                        help='Cartella del progetto STM32 (default ../<[firmware].nome_progetto>)')
    parser.add_argument('--senza-onnx', action='store_true', help='Salta l esportazione ONNX')
    args = parser.parse_args()
    check_contract(args.model.parent)
    cfg = configurazione_modello(args.model.parent)
    progetto = args.progetto or ROOT.parent / cfg.firmware.nome_progetto
    if not (progetto / 'Core/Src/main.c').exists():
        raise SystemExit(f'{progetto} non sembra un progetto STM32 (manca Core/Src/main.c).')
    destinazione = progetto / 'Core/Inc/config_controllo.h'
    destinazione.write_text(header(cfg, args.model))
    print(f'Scritto {destinazione}')
    import aggiorna_pesi_xcubeai
    aggiorna_pesi_xcubeai.main(['--model', str(args.model),
                                '--params', str(progetto / 'X-CUBE-AI/App/network_data_params.c')])
    if not args.senza_onnx:
        import esporta_onnx
        esporta_onnx.main(['--model', str(args.model),
                           '--output', str(ROOT / f'export_{args.model.parent.name}')])
    print('Fatto. In STM32CubeIDE: Project > Build Project, poi Run.')


if __name__ == '__main__':
    main()
