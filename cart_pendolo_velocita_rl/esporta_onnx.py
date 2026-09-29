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

from addestra import check_contract
from ambiente import CartPendoloEnv

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT / 'modelli/migliore.zip')
    parser.add_argument('--output', type=Path, default=ROOT / 'export_stm32')
    args = parser.parse_args()
    check_contract(args.model.parent)
    source_hash = hashlib.sha256(args.model.read_bytes()).hexdigest()
    model = SAC.load(args.model, device='cpu')
    model.policy.set_training_mode(False)
    actor = model.actor
    if (model.observation_space.shape != (6,)
            or model.action_space.shape != (1,)
            or not np.all(model.action_space.low == -1)
            or not np.all(model.action_space.high == 1)
            or actor.use_sde
            or not isinstance(actor.features_extractor, FlattenExtractor)
            or [type(layer) for layer in actor.latent_pi]
            != [torch.nn.Linear, torch.nn.ReLU, torch.nn.Linear, torch.nn.ReLU]
            or not isinstance(actor.mu, torch.nn.Linear)):
        raise ValueError('Attore incompatibile con questo esportatore 6-128-128-1.')
    layers = [actor.latent_pi[0], actor.latent_pi[2], actor.mu]
    if [(x.in_features, x.out_features) for x in layers] != [(6, 128), (128, 128), (128, 1)]:
        raise ValueError('Dimensioni degli strati inattese.')

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
        output = 'action' if i == 2 else f'hidden_{i}'
        nodes.append(helper.make_node('Gemm', [previous, weight, bias], [affine],
                                      name=f'dense_{i}', transB=1))
        nodes.append(helper.make_node('Tanh' if i == 2 else 'Relu', [affine], [output],
                                      name=f'activation_{i}'))
        previous = output
    graph = helper.make_graph(
        nodes, 'pendolo_sac_actor',
        [helper.make_tensor_value_info('observation', TensorProto.FLOAT, [1, 6])],
        [helper.make_tensor_value_info('action', TensorProto.FLOAT, [1, 1])], weights)
    exported = helper.make_model(graph, producer_name='cart_pendolo_velocita_rl',
                                 opset_imports=[helper.make_opsetid('', 13)], ir_version=7)
    helper.set_model_props(exported, {
        'checkpoint_sha256': source_hash,
        'policy': 'SAC deterministic: tanh(mu), no sampling',
        'inputs': 'x/limite,sin(theta),cos(theta),vx/vmax,omega/10,f_applied/fmax',
        'output': 'normalized signed STEP frequency request [-1,1]',
        'control_hz': '50',
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
    env = CartPendoloEnv()
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
    # I kernel float32 PyTorch/ORT accumulano i prodotti in ordini diversi.
    np.testing.assert_allclose(actual, expected, atol=1e-5, rtol=2e-5)
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
        'input': {'name': 'observation', 'shape': [1, 6]},
        'output': {'name': 'action', 'shape': [1, 1], 'range': [-1, 1]},
        'operators': [node.op_type for node in nodes],
        'parameters': sum(layer.weight.numel() + layer.bias.numel() for layer in layers),
        'file_bytes': path.stat().st_size,
        'validation_samples': len(observations), 'simulation_samples': len(rollout),
        'max_absolute_error': float(np.max(np.abs(actual - expected))),
        'mean_absolute_error': float(np.mean(np.abs(actual - expected))),
        'observed_output_min': float(actual.min()), 'observed_output_max': float(actual.max()),
        'atol': 1e-5, 'rtol': 2e-5, 'validation_passed': True,
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
