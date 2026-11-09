# Pendolo inverso su carrello — controllo con reinforcement learning su STM32

Swing-up ed equilibrio di un pendolo su carrello mosso da un motore stepper, con una rete
neurale addestrata in simulazione (SAC + MuJoCo) ed eseguita su una NUCLEO-F446RE (X-CUBE-AI).

| Cartella | Contenuto |
|---|---|
| [`cart_pendolo_velocita_rl/`](cart_pendolo_velocita_rl/) | simulazione, addestramento, valutazione ed esportazione nel firmware; rete consegnata in `modelli_finale/` |
| [`Controllo_SwingUp_Finale/`](Controllo_SwingUp_Finale/LEGGIMI.md) | progetto STM32CubeIDE del firmware: encoder, stimatore dello stato, rete (132 ingressi), comando dello stepper, homing, protocollo HIL |

Rami: `main` è la versione consegnata; `archivio-completo` contiene tutte le versioni
precedenti di firmware e reti, le configurazioni delle prove, i risultati e la variante con
motion capture/ROS.

## Firmware

Asta a 20 cm dal centro del perno al capo corto.

1. STM32CubeIDE → *File → Import → Existing Projects into Workspace* →
   `Controllo_SwingUp_Finale`.
2. *Project → Build Project*, poi *Run*. Non rigenerare il codice da CubeMX (vedi il `LEGGIMI.md` del firmware).
3. Pendolo appeso e fermo, premere **B1**: swing-up ed equilibrio. B1 ferma tutto.

## Simulazione e addestramento

Dalla cartella `cart_pendolo_velocita_rl`. `Avvia.command` (doppio clic) crea l'ambiente
Python (`.venv`, Python 3.12, `requirements.txt`) se manca e propone un menu; da terminale
esegue lo script indicato con l'ambiente giusto:

```sh
./Avvia.command                                     # menu
./Avvia.command simula.py                           # rete consegnata nella finestra MuJoCo
./Avvia.command simula.py --manual                  # finestra senza rete, comandi manuali
./Avvia.command addestra.py                         # addestra con config.toml in modelli_riaddestramento/
./Avvia.command addestra.py --resume --output modelli_riaddestramento --steps 300000
./Avvia.command simula.py valuta --model modelli_riaddestramento/migliore.zip --episodes 20
./Avvia.command esporta_firmware.py --model modelli_riaddestramento/migliore.zip
```

Poi in STM32CubeIDE: *Project → Build Project* e *Run*.

| File | Contenuto |
|---|---|
| `config.toml` | configurazione unica: modello, episodio, ricompensa, disturbi, sensori, motore, stimatore, storia, SAC, firmware |
| `cart_pendolo_asta20.xml` | modello MuJoCo del banco (asta 0,78 / 0,20 m) |
| `ambiente.py` | ambiente Gymnasium: lettura di `config.toml`, motore (azione → Hz STEP e rampa, come il firmware), stimatore dello stato (come `stima.c`), disturbi e guasti dei sensori, storia degli ingressi |
| `finestra.py` | finestre MuJoCo (rete o manuale; prova del perno) |
| `addestra.py` | SAC, checkpoint, ripresa, selezione della rete migliore |
| `simula.py` | simulazione e prove: `finestra` (default), `valuta`, `grafici`, `confronta`, `smorzamento`, `perno` (`python simula.py -h`) |
| `esporta_firmware.py` | `config_controllo.h`, pesi X-CUBE-AI, ONNX con verifica PyTorch/ONNX |
| `hil.py` | lato PC della prova hardware-in-the-loop (vedi il `LEGGIMI.md` del firmware) |
| `modelli_finale/` | rete consegnata (in `TEST_ESEGUITI.md`: `modelli_asta20_basevecchia_storia1s_20s`; `migliore.zip`), configurazione salvata (`config.json`), curva di addestramento (`progresso.csv`) |
| `TEST_ESEGUITI.md` | diario di tutte le prove (anche delle reti precedenti) |

### Pipeline

```
config.toml ──> addestra.py ──> modelli_*/migliore.zip + config.json
                                     │
                                     ├─> simula.py (finestra, valuta, grafici, confronta, smorzamento)
                                     │
                                     └─> esporta_firmware.py ──> ../Controllo_SwingUp_Finale
                                            ├─ Core/Inc/config_controllo.h   (costanti di motore, stimatore, storia, avvio)
                                            ├─ Core/Src/reti_candidate.c     (pesi e guadagni dello stimatore)
                                            ├─ X-CUBE-AI/App/                (pesi; rigenerato con stedgeai se cambiano gli ingressi)
                                            └─ export_<cartella>/pendolo_actor.onnx (verifica PyTorch/ONNX)
```

Ogni addestramento salva la configurazione completa in `<cartella>/config.json`: simulazione,
valutazione, HIL ed esportazione usano quella, quindi modificare `config.toml` non cambia le
reti già addestrate. Un checkpoint con un XML diverso viene rifiutato; se è cambiato solo
`ambiente.py` compare un avviso (errore con `--resume`). `addestra.py` non scrive in una
cartella che contiene già un addestramento (`--resume` per riprenderlo, `--output` per una
nuova cartella). Un addestramento completo (2,5 M passi) usa circa 1 GB di buffer.

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

Le misure passano dallo stimatore, che ha anche l'offset dell'encoder come terzo stato: la
rete riceve l'angolo vero stimato. I modi sono disturbi equivalenti, non un modello
strutturale dell'asta o della cinghia.

## Risultati della rete consegnata

`simula.py smorzamento`, 20 semi per valore, con disturbi:

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

## Test

I test sono locali, nella cartella `test/` (non tracciata). Dalla radice:

```sh
python test/verifica.py          # test Python e verifica geometrica
python test/test_controllo.py    # firmware: USER CODE reale con HAL simulata
python test/test_stima_c.py      # stima.c = stimatore Python
python test/test_hil.py          # protocollo HIL
python test/test_candidati.py    # pesi del firmware = checkpoint
```
