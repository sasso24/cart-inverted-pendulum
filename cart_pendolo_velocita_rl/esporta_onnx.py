"""Esporta e verifica l'attore SAC deterministico per X-CUBE-AI.

Dipendenze aggiuntive: pip install onnx onnxruntime
"""
from pathlib import Path
import argparse
import hashlib
import json

import numpy as np
import onnx
import onnxruntime as ort
import torch
from onnx import TensorProto, helper, numpy_helper
from stable_baselines3 import SAC
from stable_baselines3.common.torch_layers import FlattenExtractor

from addestra import check_contract, profilo_salvato
from disturbi import crea_ambiente
from configurazione import configurazione_ambiente, configurazione_modello

ROOT = Path(__file__).resolve().parent


def strati_attore(model):
    """Strati lineari dell'attore deterministico: n -> (ReLU)* -> 1 tanh.

    n = 6 ingressi attuali + 7 per ogni ritardo della storia ([storia], storia.py).
    """
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT / 'modelli_asta20_basevecchia_storia1s_20s/migliore.zip')
    parser.add_argument('--output', type=Path, default=ROOT / 'export_stm32')
    args = parser.parse_args(argv)
    check_contract(args.model.parent)
    source_hash = hashlib.sha256(args.model.read_bytes()).hexdigest()
    scala_omega = configurazione_modello(args.model.parent).episodio.scala_omega_rad_s
    periodo = configurazione_modello(args.model.parent).episodio.periodo_controllo_s
    model = SAC.load(args.model, device='cpu')
    model.policy.set_training_mode(False)
    layers = strati_attore(model)
    n = layers[0].in_features
    ritardi = configurazione_modello(args.model.parent).storia.ritardi
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
    # Rollout con lo stesso profilo dell'addestramento: ingressi stimati da stima.py.
    cfg = configurazione_ambiente(args.model.parent)
    env = crea_ambiente(profilo_salvato(args.model.parent), configurazione=cfg)
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
    if hashlib.sha256(args.model.read_bytes()).hexdigest() != source_hash:
        raise RuntimeError('Checkpoint cambiato durante l’esportazione: riprovare.')

    args.output.mkdir(parents=True, exist_ok=True)
    path = args.output / 'pendolo_actor.onnx'
    onnx.save(exported, path)
    np.savetxt(args.output / 'validation_inputs.csv', observations, delimiter=',', fmt='%.9g')
    np.savetxt(args.output / 'validation_outputs.csv', expected, delimiter=',', fmt='%.9g')
    config = json.loads((args.model.parent / 'config.json').read_text())
    report = {
        'checkpoint': str(args.model.resolve()), 'checkpoint_sha256': source_hash,
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
    (args.output / 'export_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'Esportato: {path}\nVerifica: {len(observations)} ingressi, '
          f'errore massimo {report["max_absolute_error"]:.3g}')


if __name__ == '__main__':
    main()
