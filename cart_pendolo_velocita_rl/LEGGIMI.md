# Addestramento RL del pendolo inverso su carrello (SAC + MuJoCo)

Addestra con Soft Actor-Critic (stable-baselines3) la rete che fa **swing-up ed equilibrio**
del pendolo sul carrello mosso da uno stepper, e la porta nel firmware STM32
`../Controllo_SwingUp_Asta20_Storia_BaseVecchia`.

Rete consegnata: `modelli_asta20_basevecchia_storia1s_20s/migliore.zip`, addestrata con
`config.toml` (seed 42, episodi da 20 s, checkpoint a 1,78 M passi su 2,5 M). È la rete
caricata nel firmware: `../Controllo_SwingUp_Asta20_Storia_BaseVecchia/tests/test_candidati.py`
verifica che i pesi siano identici al checkpoint.

## Avvio

```sh
./Setup.command          # crea .venv e installa requirements.txt (Python 3.12)
./Verifica.command       # test e verifica geometrica senza finestra
python -m unittest test_configurazione test_disturbi test_periodo test_stima test_storia test_versione
```

I launcher `.command` usano `.venv` di questa cartella (o, se manca, `../cart_pendolo_stepper_rl/.venv`).

| Comando | Cosa fa |
|---|---|
| `./Addestra.command` | addestra con `config.toml` in `modelli_riaddestramento/` |
| `./Avvia_RL.command` | finestra MuJoCo con la rete consegnata (`--model` per un'altra) |
| `./Avvia_Manuale.command` | finestra senza rete, comandi manuali |
| `./Valuta.command --episodes 20` | valutazione su semi separati (2000 e successivi), nominale e con disturbi |
| `./Grafici.command` | CSV e grafici della traiettoria (Hz, posizione, velocità) |
| `./Esporta_Firmware.command` | scrive configurazione e pesi nel progetto STM32 |
| `./HIL.command` | prova hardware-in-the-loop con la scheda (vedi `../Controllo_SwingUp_Asta20_Storia_BaseVecchia/HIL.md`) |

Altri strumenti: `confronta.py` (più reti sugli stessi semi in più condizioni),
`prova_smorzamento.py` (sweep dello smorzamento del perno), `prova_perno.py` (pendolo libero
con smorzamento e attrito regolabili dalla finestra, per confrontare il perno simulato con il banco).

## Pipeline

```
config.toml ──> addestra.py ──> modelli_*/migliore.zip + config.json
                                     │
                                     ├─> valuta.py / simula.py / grafici.py / confronta.py
                                     │
                                     └─> esporta_firmware.py ──> ../Controllo_SwingUp_Asta20_Storia_BaseVecchia
                                            ├─ Core/Inc/config_controllo.h   (costanti di motore, stimatore, storia, avvio)
                                            ├─ Core/Src/reti_candidate.c     (pesi e guadagni dello stimatore)
                                            ├─ X-CUBE-AI/App/                (pesi; rigenerato con stedgeai se cambiano gli ingressi)
                                            └─ export_<cartella>/pendolo_actor.onnx (verifica PyTorch/ONNX)
```

Ogni addestramento salva la configurazione completa in `<cartella>/config.json` (e il TOML in
`config_usata.toml`): simulazione, valutazione, HIL ed esportazione usano quella, quindi
modificare `config.toml` non cambia le reti già addestrate. Un checkpoint con XML o parametri
fisici diversi viene rifiutato. `addestra.py` non scrive in una cartella che contiene già un
addestramento (`--resume` per riprenderlo, `--output` per una nuova cartella).

```sh
./Addestra.command                                         # 2,5 M passi, ~1 GB di buffer (132 ingressi)
./Addestra.command --resume --output modelli_riaddestramento --steps 300000
./Valuta.command --model modelli_riaddestramento/migliore.zip --episodes 20
python esporta_firmware.py --model modelli_riaddestramento/migliore.zip
```

Poi in STM32CubeIDE: *Project → Build Project* e *Run*.

## File

| File | Contenuto |
|---|---|
| `config.toml` | configurazione unica: episodio, ricompensa, disturbi, sensori, motore, stimatore, storia, SAC, firmware |
| `cart_pendolo_asta20.xml` | modello MuJoCo del banco (asta 0,78 / 0,20 m) usato dalla rete |
| `cart_pendolo.xml` | geometria originale (default di `ambiente.py`, usata dai test) |
| `ambiente.py` | ambiente Gymnasium: osservazioni, azione, ricompensa, finecorsa, reset |
| `disturbi.py`, `disturbi.toml` | vibrazioni, variazioni dei parametri, rumore e guasti dei sensori, ritardo |
| `stepper.py`, `stepper.toml` | conversione azione → Hz STEP e rampa di frequenza (come il firmware) |
| `stima.py`, `stimatore.toml` | stimatore di angolo, velocità angolare e offset dell'encoder (come `stima.c`) |
| `storia.py` | ingressi passati della rete (sezione `[storia]`) |
| `configurazione.py` | lettura e validazione di `config.toml` |
| `addestra.py` | SAC, checkpoint, ripresa, selezione della rete migliore |
| `valuta.py`, `simula.py`, `finestra.py`, `grafici.py`, `confronta.py` | valutazione e visualizzazione |
| `esporta_firmware.py`, `esporta_onnx.py`, `aggiorna_pesi_xcubeai.py` | esportazione nel firmware |
| `hil.py` | lato PC della prova hardware-in-the-loop |
| `test_*.py`, `verifica.py` | test automatici |
| `modelli_asta20_basevecchia_storia1s_20s/` | rete consegnata, configurazione salvata, curve (`progresso.csv`) |
| `export_modelli_asta20_basevecchia_storia1s_20s/` | ONNX della rete consegnata e report della verifica |
| `log_asta/`, `risultati/` | log dell'addestramento e prove della rete consegnata |
| `TEST_ESEGUITI.md` | diario di tutte le prove (anche delle reti precedenti) |

## Rete, ingressi e azione

Rete **132 → 128 ReLU → 128 ReLU → 1 tanh**. L'uscita u ∈ [-1, 1] è la velocità del carrello:
il segno sceglie il verso (DIR), il modulo la frequenza STEP, `f = u × 32 000 Hz`
(0,48 m/s con 200 passi/giro, microstepping 32, cinghia 2 mm, puleggia 48 denti). La frequenza
applicata segue una rampa di 1,6 MHz/s; all'inversione la rampa resta ferma a zero 2 ms,
identica al firmware.

Ingressi attuali (sei, stimati, non derivati dalle misure):

1. posizione del carrello / limite di corsa (0,40 m);
2. sin(angolo), 3. cos(angolo) (0 = pendolo in alto);
4. velocità del carrello / velocità massima;
5. velocità angolare / 10 rad/s;
6. frequenza applicata dopo la rampa / frequenza massima.

Storia: per ogni ritardo k = 1…10 (ogni 20 ms fino a 0,2 s) e 15, 20, …, 50 (ogni 100 ms fino a
1 s) i sei ingressi di k periodi fa e l'azione decisa allora: 6 + 7 × 18 = 132 ingressi.

Fisica a 500 Hz, rete a 50 Hz (20 ms), episodi di 20 s. L'episodio parte appeso (-180°);
il 20% degli episodi di addestramento parte in alto per allenare presto l'equilibrio.
Successo = in equilibrio senza interruzioni da 12 s alla fine.

## Ricompensa

`r = alto · F_x · F_ω · F_angolo · F_v + peso_energia · exp(-((E-1)/scala)²) - penalità sul comando - penalità di bordo`

con `alto = (1 + cos θ)/2` e ogni `F = minimo + (1 - minimo) · exp(-(valore/scala)²)`.
La priorità è sulla velocità angolare (termine stretto e pesante); il termine sull'angolo è
largo, così qualche grado di errore dell'encoder costa poco. L'aiuto energetico guida lo
swing-up; penalità su ampiezza, variazione e inversioni del comando e vicino ai bordi.
Valori e commenti in `config.toml` → `[ricompensa]`.

## Disturbi e guasti simulati

- Tre modi oscillanti smorzati (cinghia 8-18 Hz, motore 20-40 Hz, asta 3-10 Hz) eccitati
  dall'accelerazione del motore e da rumore correlato.
- Variazioni per episodio di masse, smorzamenti, guadagno del motore, ritardo del comando (0, 2 o 4 ms).
- **Perno del pendolo randomizzato**: viscoso log-uniforme 0,0003-0,015 N·m·s, coulombiano
  0-0,006 N·m, resistenza dell'aria 0-6,6e-4 N·m·s². Il perno del banco non è misurato:
  l'intervallo largo copre sia un cuscinetto che frena poco sia la stima del banco.
- Rumore di posizione 5-10 mm e dell'encoder 0,5-1°, quantizzazione a 2400 conteggi/giro.
- Guasti dell'encoder visti sul banco: offset iniziale fino a ±3°, conteggi persi (deriva),
  impulsi spuri; passi persi del motore.

Le misure passano dallo stimatore (`stima.py`), che ha anche l'offset dell'encoder come terzo
stato: la rete riceve l'angolo vero stimato. I modi sono disturbi equivalenti, non un modello
strutturale dell'asta o della cinghia.

## Risultati della rete consegnata

`prova_smorzamento.py`, 20 semi per valore, con disturbi (`risultati/prova_basevecchia_storia_*.json`):

| perno | successi |
|---|---:|
| 0,0003-0,005 N·m·s, attrito 0,0005 N·m | 80 / 80 |
| 0,0003-0,005 N·m·s, senza attrito coulombiano | 80 / 80 |
| stima banco 0,0095 N·m·s + 0,006 N·m | 16 / 20 (nominale 18 / 20) |

In alto in 5,5-7,5 s (10-11 s con il perno della stima banco). Sul banco (30/10) lo swing-up
con aggancio funziona. Dettagli in `TEST_ESEGUITI.md`.

## Finestra

R reset, C alterna SAC/arresto, A/D spinte, frecce movimento manuale di ±10 cm, Spazio pausa,
Esc chiude. Mouse per ruotare, rotella per lo zoom.
