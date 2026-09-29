# Attore SAC per X-CUBE-AI

Importare **pendolo_actor.onnx** come modello ONNX in X-CUBE-AI.
Selezionare STM32F446RE, eseguire Analyze, Validate e generare il codice.
Il file contiene tutti i pesi: non servono file esterni.

- Origine: `cart_pendolo_velocita_rl/modelli/migliore.zip`.
- Rete deterministica: 6 → 128 ReLU → 128 ReLU → 1 tanh.
- Float32, batch fisso 1, ONNX opset 13, IR 7.
- Operatori: Gemm, Relu, Gemm, Relu, Gemm, Tanh.
- Ingresso `observation`: forma `[1, 6]`.
- Uscita `action`: forma `[1, 1]`, azione normalizzata in [-1, 1].
- 17.537 parametri; esclusi critici, campionamento e ramo log_std.

## Ingressi e uscita

Preparare i sei float nell'ordine seguente, ogni 20 ms:

| Indice | Valore |
|---|---|
| 0 | posizione carrello in metri, zero al centro / 0,462 m |
| 1 | sin(theta) |
| 2 | cos(theta) |
| 3 | velocità carrello in m/s / 0,48 m/s |
| 4 | velocità angolare in rad/s / 10 rad/s |
| 5 | frequenza STEP firmata applicata dopo la rampa / 32.000 Hz |

Theta è in radianti: zero con pendolo in alto, pi greco in basso.
Le normalizzazioni, la lettura dei sensori e la conversione in Hz sono esterne
al grafo ONNX. La tanh finale è già inclusa: non applicarla di nuovo.

Con la configurazione salvata durante l'addestramento:

```text
u = clip(action[0], -1, +1)
f_richiesta = 0                                      se u == 0
f_richiesta = segno(u) * (3200 + abs(u) * 28800)       altrimenti
```

Ogni 2 ms aggiornare la rampa firmata:

```text
f_applicata += clip(f_richiesta - f_applicata, -3200, +3200)
```

La rampa configurata è 1.600.000 Hz/s. La richiesta rimane costante tra due
inferenze. Il modulo della frequenza applicata comanda STEP, il segno comanda
DIR; a zero non si generano impulsi.

## Verifica e riproduzione

`export_report.json` contiene hash del checkpoint e dell'ONNX, configurazione
di addestramento, versioni degli strumenti ed errori numerici misurati.
Il confronto usa Stable Baselines3 `predict(..., deterministic=True)` e
ONNX Runtime su 5.100 ingressi, di cui 1.000 ricavati dalla simulazione.
`validation_inputs.csv` contiene sei colonne senza intestazione;
`validation_outputs.csv` contiene le corrispondenti uscite di riferimento.
La verifica con il compilatore ST e sulla scheda resta da eseguire in X-CUBE-AI.

Per riesportare dalla cartella principale del repository:

```sh
cart_pendolo_stepper_rl/.venv/bin/python cart_pendolo_velocita_rl/esporta_onnx.py
```

L'esportatore richiede l'ambiente di addestramento più `onnx` e `onnxruntime`.
Accetta `--model percorso/checkpoint.zip` e `--output cartella` e verifica
la corrispondenza tra configurazione salvata e ambiente corrente.
