"""Porta una rete addestrata nel progetto STM32 (default ../Controllo_SwingUp_Finale).

Un solo comando, sempre dalla configurazione SALVATA col modello:
  1. Core/Inc/config_controllo.h: motore, rampa, scale degli ingressi, limite
     di corsa, stimatore, swing-up, controlli dell'encoder;
  2. X-CUBE-AI/App/network_data_params.c: pesi della rete (stessa forma
     del codice X-CUBE-AI gia generato; con un numero di ingressi diverso,
     per esempio con la storia, X-CUBE-AI/App viene rigenerato con stedgeai);
  3. export_<cartella>/pendolo_actor.onnx con la verifica PyTorch/ONNX.

    python esporta_firmware.py
    python esporta_firmware.py --model modelli_finale/migliore.zip --progetto ../Controllo_SwingUp_Finale
    python esporta_firmware.py --model a/migliore.zip --candidato b/migliore.zip
        (piu reti nello stesso firmware, scelte al banco con rete_scelta)
    python esporta_firmware.py --solo-onnx     (solo ONNX e verifica, firmware invariato)

Dopo: in STM32CubeIDE Project > Build Project e caricare sulla scheda.
Dipendenze aggiuntive per l'ONNX: pip install onnx onnxruntime
"""
import argparse
from dataclasses import replace
from datetime import date
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

from addestra import check_contract
from ambiente import ROOT, configurazione_modello, con_offset, crea_ambiente

NOME_ARRAY = 's_network_weights_array_u64'


# =============================================================================
#  Attore SAC -> ONNX, con verifica PyTorch/ONNX
# =============================================================================

def strati_attore(model):
    """Strati lineari dell'attore deterministico: n -> (ReLU)* -> 1 tanh.

    n = 6 ingressi attuali + 7 per ogni ritardo della storia ([storia]).
    """
    import torch
    from stable_baselines3.common.torch_layers import FlattenExtractor
    actor = model.actor
    tipi = [type(layer) for layer in actor.latent_pi]
    n = model.observation_space.shape[0] if len(model.observation_space.shape) == 1 else 0
    if (n < 6 or (n - 6) % 7
            or model.action_space.shape != (1,)
            or not np.all(model.action_space.low == -1)
            or not np.all(model.action_space.high == 1)
            or actor.use_sde
            or not isinstance(actor.features_extractor, FlattenExtractor)
            or tipi != [torch.nn.Linear, torch.nn.ReLU] * (len(tipi) // 2)
            or not isinstance(actor.mu, torch.nn.Linear)):
        raise ValueError('Attore incompatibile: serve 6 (+7*k) -> Linear/ReLU ... -> 1 tanh.')
    layers = list(actor.latent_pi[::2]) + [actor.mu]
    if layers[0].in_features != n or layers[-1].out_features != 1 or any(
            a.out_features != b.in_features for a, b in zip(layers, layers[1:])):
        raise ValueError('Dimensioni degli strati inattese.')
    return layers



def esporta_onnx(modello, uscita):
    """export_<cartella>/pendolo_actor.onnx, ingressi/uscite di verifica e export_report.json."""
    import onnx
    import onnxruntime as ort
    from onnx import TensorProto, helper, numpy_helper
    import torch
    from stable_baselines3 import SAC
    modello, uscita = Path(modello), Path(uscita)
    check_contract(modello.parent)
    source_hash = hashlib.sha256(modello.read_bytes()).hexdigest()
    cfg = configurazione_modello(modello.parent)
    scala_omega = cfg.episodio.scala_omega_rad_s
    periodo = cfg.episodio.periodo_controllo_s
    model = SAC.load(modello, device='cpu')
    model.policy.set_training_mode(False)
    layers = strati_attore(model)
    n = layers[0].in_features
    ritardi = cfg.storia.ritardi
    if n != 6 + 7*len(ritardi):
        raise ValueError(f'Rete con {n} ingressi, [storia] salvata con {len(ritardi)} ritardi.')
    # Grafo esplicito: evita operatori di campionamento SAC e dimensioni dinamiche.
    nodes, weights = [], []
    previous = 'observation'
    for i, layer in enumerate(layers):
        weight, bias = f'weight_{i}', f'bias_{i}'
        weights.extend([
            numpy_helper.from_array(layer.weight.detach().cpu().numpy().copy(), weight),
            numpy_helper.from_array(layer.bias.detach().cpu().numpy().copy(), bias),
        ])
        affine = f'affine_{i}'
        ultimo = i == len(layers) - 1
        output = 'action' if ultimo else f'hidden_{i}'
        nodes.append(helper.make_node('Gemm', [previous, weight, bias], [affine],
                                      name=f'dense_{i}', transB=1))
        nodes.append(helper.make_node('Tanh' if ultimo else 'Relu', [affine], [output],
                                      name=f'activation_{i}'))
        previous = output
    graph = helper.make_graph(
        nodes, 'pendolo_sac_actor',
        [helper.make_tensor_value_info('observation', TensorProto.FLOAT, [1, n])],
        [helper.make_tensor_value_info('action', TensorProto.FLOAT, [1, 1])], weights)
    exported = helper.make_model(graph, producer_name='cart_pendolo_velocita_rl',
                                 opset_imports=[helper.make_opsetid('', 13)], ir_version=7)
    helper.set_model_props(exported, {
        'checkpoint_sha256': source_hash,
        'policy': 'SAC deterministic: tanh(mu), no sampling',
        'inputs': f'x/limite,sin(theta),cos(theta),vx/vmax,omega/{scala_omega:g},f_applied/fmax'
                  + (f'; poi per i ritardi {list(ritardi)}: stessi sei ingressi + azione' if ritardi else ''),
        'architecture': '-'.join(str(k) for k in [n] + [l.out_features for l in layers]),
        'output': 'normalized signed STEP frequency request [-1,1]',
        'control_hz': f'{1. / periodo:g}',
    })
    onnx.checker.check_model(exported, full_check=True)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(exported.SerializeToString(), sess_options=options,
                                   providers=['CPUExecutionProvider'])

    # Stati normalizzati sintetici, casi limite e osservazioni dalla simulazione.
    rng = np.random.default_rng(42)
    observations = rng.uniform(-2, 2, (4096, 6)).astype(np.float32)
    angles = rng.uniform(-np.pi, np.pi, 4096)
    observations[:, 1] = np.sin(angles)
    observations[:, 2] = np.cos(angles)
    observations[:, 5] = rng.uniform(-1, 1, 4096)
    boundaries = np.array([[0, 0, 1, 0, 0, 0], [0, 0, -1, 0, 0, 0],
                           [1, 0, 1, 1, 1, 1], [-1, 0, 1, -1, -1, -1]], dtype=np.float32)
    if ritardi:
        # Storia sintetica: per ogni ritardo sei ingressi come sopra e un'azione in [-1, 1].
        blocchi = []
        for _ in ritardi:
            b = rng.uniform(-2, 2, (4096, 7)).astype(np.float32)
            a = rng.uniform(-np.pi, np.pi, 4096)
            b[:, 1], b[:, 2] = np.sin(a), np.cos(a)
            b[:, 5:7] = rng.uniform(-1, 1, (4096, 2))
            blocchi.append(b)
        observations = np.concatenate([observations] + blocchi, axis=1)
        boundaries = np.concatenate([boundaries] + [np.c_[boundaries, boundaries[:, 5:6]]]*len(ritardi), axis=1)
    # Rollout con lo stesso profilo dell'addestramento: ingressi stimati dallo stimatore.
    env = crea_ambiente(cfg.disturbi, configurazione=cfg)
    rollout = []
    try:
        obs, _ = env.reset(seed=2026)
        for i in range(1000):
            rollout.append(obs.copy())
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                obs, _ = env.reset(seed=2027 + i)
    finally:
        env.close()
    observations = np.concatenate([boundaries, observations, np.asarray(rollout)])
    # Confronto a batch fisso 1, identico all'interfaccia destinata al firmware.
    expected = np.asarray([model.predict(x, deterministic=True)[0] for x in observations])
    actual = np.concatenate([session.run(['action'], {'observation': x[None]})[0]
                             for x in observations])
    # I kernel float32 PyTorch/ORT accumulano i prodotti in ordini diversi: l'errore
    # cresce circa con la radice del numero di ingressi (tolleranze storiche a 6).
    scala = max(1., n / 6.)**0.5
    atol, rtol = 1e-5*scala, 2e-5*scala
    np.testing.assert_allclose(actual, expected, atol=atol, rtol=rtol)
    # La tanh SIMD può superare 1 di un ulp per arrotondamento float32.
    if not np.isfinite(actual).all() or np.any(np.abs(actual) > 1 + 2 * np.finfo(np.float32).eps):
        raise ValueError('Uscite ONNX non valide.')
    if hashlib.sha256(modello.read_bytes()).hexdigest() != source_hash:
        raise RuntimeError('Checkpoint cambiato durante l’esportazione: riprovare.')

    uscita.mkdir(parents=True, exist_ok=True)
    path = uscita / 'pendolo_actor.onnx'
    onnx.save(exported, path)
    np.savetxt(uscita / 'validation_inputs.csv', observations, delimiter=',', fmt='%.9g')
    np.savetxt(uscita / 'validation_outputs.csv', expected, delimiter=',', fmt='%.9g')
    config = json.loads((modello.parent / 'config.json').read_text())
    report = {
        'checkpoint': str(modello.resolve()), 'checkpoint_sha256': source_hash,
        'onnx_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'opset': 13, 'ir_version': 7, 'dtype': 'float32',
        'input': {'name': 'observation', 'shape': [1, n], 'storia_ritardi': list(ritardi)},
        'output': {'name': 'action', 'shape': [1, 1], 'range': [-1, 1]},
        'operators': [node.op_type for node in nodes],
        'parameters': sum(layer.weight.numel() + layer.bias.numel() for layer in layers),
        'file_bytes': path.stat().st_size,
        'validation_samples': len(observations), 'simulation_samples': len(rollout),
        'max_absolute_error': float(np.max(np.abs(actual - expected))),
        'mean_absolute_error': float(np.mean(np.abs(actual - expected))),
        'observed_output_min': float(actual.min()), 'observed_output_max': float(actual.max()),
        'atol': atol, 'rtol': rtol, 'validation_passed': True,
        'xcubeai_validation': 'Not run; import and Analyze/Validate in X-CUBE-AI.',
        'versions': {'torch': torch.__version__, 'onnx': onnx.__version__,
                     'onnxruntime': ort.__version__},
        'training_config': config,
    }
    (uscita / 'export_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'Esportato: {path}\nVerifica: {len(observations)} ingressi, '
          f'errore massimo {report["max_absolute_error"]:.3g}')


# =============================================================================
#  Pesi nel codice X-CUBE-AI gia generato (network_data_params.c)
# =============================================================================
# Il codice generato da X-CUBE-AI (network.c) resta invariato: cambia solo
# l'array dei pesi in network_data_params.c. Vale per la rete n-128-128-1
# generata con `--compression none`: float32 in ordine
# W0[128xn], b0, W1[128x128], b1, W2[1x128], b2, piu un float di riempimento.
# L'ordine e stato ricavato e verificato decodificando i pesi della rete
# precedente (identici bit per bit al checkpoint esportato il 29/09).

def pesi_rete(model_path):
    from stable_baselines3 import SAC
    layers = strati_attore(SAC.load(model_path, device='cpu'))
    parti = []
    for layer in layers:
        parti += [layer.weight.detach().numpy().ravel(), layer.bias.detach().numpy()]
    return np.concatenate(parti).astype('<f4')


def leggi_array(testo):
    inizio = testo.index(NOME_ARRAY)
    corpo = testo[testo.index('{', inizio) + 1:testo.index('};', inizio)]
    valori = [int(h, 16) for h in re.findall(r'0x([0-9a-fA-F]+)U', corpo)]
    return inizio, np.frombuffer(np.array(valori, dtype='<u8').tobytes(), dtype='<f4')


def scrivi_array(testo, pesi):
    inizio, attuali = leggi_array(testo)
    if len(attuali) != len(pesi) + 1:
        raise ValueError(f'Dimensione diversa: firmware {len(attuali)} float, rete {len(pesi)}+1. '
                         'La rete ha strati diversi da quella generata con X-CUBE-AI: importare '
                         'export_<cartella>/pendolo_actor.onnx in STM32CubeMX (X-CUBE-AI) e rigenerare.')
    dati = np.concatenate([pesi, np.zeros(1, '<f4')])
    parole = np.frombuffer(dati.tobytes(), dtype='<u8')
    righe = []
    for k in range(0, len(parole), 4):
        # Stesso formato del generatore ST: esadecimale minimo, virgola finale.
        righe.append('  ' + ', '.join(f'0x{int(w):x}U' for w in parole[k:k+4]) + ',')
    apertura = testo.index('{', inizio) + 1
    chiusura = testo.index('};', inizio)
    return testo[:apertura] + '\n' + '\n'.join(righe) + '\n' + testo[chiusura:]



def aggiorna_pesi(modello, params):
    pesi = pesi_rete(modello)
    # newline='' conserva le terminazioni di riga originali (LF o CRLF).
    with params.open(newline='') as stream:
        testo = scrivi_array(stream.read(), pesi)
    _, riletti = leggi_array(testo)
    if not np.array_equal(riletti[:len(pesi)], pesi):
        raise RuntimeError('Verifica fallita dopo la scrittura.')
    with params.open('w', newline='') as stream:
        stream.write(testo)
    print(f'Aggiornati {len(pesi)} pesi in {params}')


# =============================================================================
#  Header del firmware e reti candidate
# =============================================================================

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
        '/* Motore: stessi valori di [stepper] (ambiente.py). */',
        f'#define CFG_MICROSTEP             {sp.microstepping}',
        f'#define CFG_FREQ_MIN_HZ           {_f(sp.frequenza_min_hz)}',
        f'#define CFG_FREQ_MAX_HZ           {_f(sp.frequenza_max_hz)}',
        f'#define CFG_RAMPA_HZ_S            {_f(sp.rampa_hz_s)}',
        f'#define CFG_ZONA_MORTA            {_f(sp.zona_morta)}',
        f'#define CFG_METRI_PER_STEP        {_f(sp.metri_per_impulso)}',
        '/* Inversione: periodi di rampa (2 ms) con frequenza ferma a zero (ambiente.py). */',
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
        '/* Storia degli ingressi ([storia], ambiente.py): per ogni ritardo (in periodi)',
        ' * i sei ingressi di allora e l azione decisa, dopo i sei ingressi attuali. */',
        f'#define CFG_STORIA_N              {len(cfg.storia.ritardi)}U',
        f'#define CFG_STORIA_INGRESSI       {cfg.storia.ingressi}U']
    if cfg.storia.ritardi:
        righe += [f'#define CFG_STORIA_MAX            {max(cfg.storia.ritardi)}U',
                  '#define CFG_STORIA_RITARDI        { ' + ', '.join(f'{k}U' for k in cfg.storia.ritardi) + ' }']
    righe += ['', '#endif', '']
    return '\n'.join(righe)


def configurazione_esportata(modello, polo_offset=None):
    """Configurazione salvata con la rete, con lo stimatore dell offset aggiunto se richiesto."""
    check_contract(Path(modello).parent)
    cfg = configurazione_modello(Path(modello).parent)
    if polo_offset is not None:
        if cfg.stimatore.l_b:
            raise SystemExit(f'{modello}: la rete ha gia lo stimatore dell offset.')
        cfg = con_offset(cfg, polo_offset)
        st = cfg.stimatore
        print(f'{modello}: stimatore con offset l_theta {st.l_theta:.6f}, l_omega {st.l_omega:.6f}, l_b {st.l_b:.6f}')
    return cfg


# #define che possono cambiare da una rete all altra: guadagni dello stimatore
# (tabella in reti_candidate.c) e tick dell inversione (vale il massimo).
PER_RETE = ('CONFIG_CONTROLLO_H', 'CFG_TICK_FERMO_INVERSIONE',
            'STIMA_L_THETA', 'STIMA_L_OMEGA', 'STIMA_L_B', 'STIMA_L_X')


def _differenze(testo_a, testo_b, tolleranza=1e-5):
    """#define diverse tra due header (numeri confrontati con tolleranza relativa;
    il tick dell inversione e escluso, vale il massimo)."""
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


def ingressi_xcubeai(progetto):
    """Ingressi della rete per cui e generato il codice X-CUBE-AI del progetto."""
    testo = (progetto / 'X-CUBE-AI/App/network.h').read_text()
    return int(re.search(r'#define AI_NETWORK_IN_1_SIZE\s+\((\d+)\)', testo)[1])


def stedgeai():
    """Percorso di stedgeai (X-CUBE-AI): variabile STEDGEAI o pacchetto in ~/STM32Cube."""
    import os
    if os.environ.get('STEDGEAI'):
        return Path(os.environ['STEDGEAI'])
    trovati = sorted(Path.home().glob('STM32Cube/Repository/Packs/STMicroelectronics/X-CUBE-AI/*/Utilities/*/stedgeai'))
    if not trovati:
        raise SystemExit('stedgeai non trovato: installare X-CUBE-AI o impostare STEDGEAI.')
    return trovati[-1]


FILE_XCUBEAI = ('network.c', 'network.h', 'network_config.h', 'network_data.c', 'network_data.h',
                'network_data_params.c', 'network_data_params.h', 'network_generate_report.txt')


def genera_xcubeai(onnx, progetto):
    """Rigenera X-CUBE-AI/App con le stesse opzioni del progetto (29/09):
    generate --target stm32f4 --name network --compression none."""
    import shutil
    import subprocess
    import tempfile
    with tempfile.TemporaryDirectory(prefix='stedgeai-') as tmp:
        tmp = Path(tmp)
        subprocess.run([str(stedgeai()), 'generate', '--target', 'stm32f4', '--name', 'network',
                        '-m', str(onnx), '--compression', 'none', '--verbosity', '1',
                        '--workspace', str(tmp / 'ws'), '--output', str(tmp / 'out')],
                       check=True, stdout=subprocess.DEVNULL)
        for nome in FILE_XCUBEAI:
            shutil.copyfile(tmp / 'out' / nome, progetto / 'X-CUBE-AI/App' / nome)
    print(f'Rigenerato X-CUBE-AI/App da {onnx}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', type=Path, default=ROOT / 'modelli_finale/migliore.zip',
                        help='Rete 0 (quella caricata all accensione)')
    parser.add_argument('--progetto', type=Path,
                        help='Cartella del progetto STM32 (default ../<[firmware].nome_progetto>)')
    parser.add_argument('--senza-onnx', action='store_true', help='Salta l esportazione ONNX')
    parser.add_argument('--solo-onnx', action='store_true',
                        help='Solo ONNX e verifica in export_<cartella>, firmware invariato')
    parser.add_argument('--stimatore-offset', type=float, metavar='POLO_1_S',
                        help='Per reti addestrate senza: firmware con lo stimatore dell offset '
                             '(polo in 1/s, es. -1). Guadagni da stima.guadagni_con_offset.')
    parser.add_argument('--candidato', action='append', default=[], metavar='RETE.zip[@POLO]',
                        help='Altre reti nello stesso firmware (rete 1, 2, ...), scelte con '
                             'rete_scelta dal debugger. @POLO: stimatore dell offset aggiunto. '
                             'Tutte devono avere la stessa configurazione del firmware.')
    args = parser.parse_args()
    if args.solo_onnx:
        esporta_onnx(args.model, ROOT / f'export_{Path(args.model).parent.name}')
        return
    cfg = configurazione_esportata(args.model, args.stimatore_offset)
    candidati = []
    for voce in args.candidato:
        percorso, _, polo = voce.partition('@')
        candidati.append((Path(percorso), configurazione_esportata(percorso, float(polo) if polo else None)))
    tick = max([cfg.stepper.tick_fermo_inversione] + [c.stepper.tick_fermo_inversione for _, c in candidati])
    if tick != cfg.stepper.tick_fermo_inversione:
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
    onnx_esportati = set()
    if ingressi_xcubeai(progetto) != cfg.storia.ingressi:
        # Numero di ingressi diverso (storia): il codice X-CUBE-AI va rigenerato dall ONNX.
        uscita = ROOT / f'export_{Path(args.model).parent.name}'
        esporta_onnx(args.model, uscita)
        onnx_esportati.add(Path(args.model))
        genera_xcubeai(uscita / 'pendolo_actor.onnx', progetto)
    aggiorna_pesi(args.model, progetto / 'X-CUBE-AI/App/network_data_params.c')

    def nome(percorso, polo):
        try:
            testo_nome = Path(percorso).resolve().relative_to(ROOT).as_posix()
        except ValueError:
            testo_nome = Path(percorso).name
        return testo_nome + (' + stimatore offset' if polo else '')
    nomi = [nome(args.model, args.stimatore_offset is not None)]
    nomi += [nome(p, '@' in v) for (p, _), v in zip(candidati, args.candidato)]
    scrivi_candidati(progetto, nomi, [pesi_rete(p) for p, _ in candidati],
                     [cfg.stimatore] + [c.stimatore for _, c in candidati])
    for k, n in enumerate(nomi):
        print(f'  rete {k}: {n}')
    if not args.senza_onnx:
        for percorso in [args.model] + [p for p, _ in candidati]:
            if Path(percorso) in onnx_esportati:
                continue
            esporta_onnx(percorso, ROOT / f'export_{Path(percorso).parent.name}')
    print('Fatto. In STM32CubeIDE: Project > Build Project, poi Run.')



if __name__ == '__main__':
    main()
