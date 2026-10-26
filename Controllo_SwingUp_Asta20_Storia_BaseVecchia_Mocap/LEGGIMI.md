# Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap — swing-up con l'angolo dal sistema di tracciamento

**Copia di `Controllo_SwingUp_Asta20_Storia_BaseVecchia` (26/10) con la stessa rete, gli
stessi pesi e lo stesso `config_controllo.h`. Cambia solo da dove arriva l'angolo del
pendolo: non più dall'encoder ma dal sistema di tracciamento del laboratorio (marker sull'asta),
tramite un nodo ROS 2 che lo manda sulla USART2 a 200 Hz.** Tutto il resto (homing,
stimatore, storia, rampa, protocollo HIL) è invariato. Collegamento, protocollo, nodi ROS e
prove: **`MOCAP.md`**. L'encoder, se montato, resta letto solo per confronto
(`encoder_theta_deg`, anche nella telemetria).

Il resto di questo file descrive la rete e vale com'era per `_Storia_BaseVecchia`.

Copia di `Controllo_SwingUp_Asta20_Storia` (24/10) con una **rete nuova addestrata da zero**
sulla configurazione della rete del 23/10 (XML con perno 0,01 N·m·s, `STIMA_K_D` 0,403, storia
di 1 s), ma con il perno del pendolo **randomizzato a ogni episodio su un intervallo largo**
(vedi "La rete" sotto). `config_controllo.h` è uguale a quello di `Controllo_SwingUp_Asta20_Storia`;
cambiano `reti_candidate.c` e i pesi in `X-CUBE-AI/App`.

Origine: copia di `Controllo_SwingUp_Asta20`: stessa asta spostata
(**0,78 m / 0,20 m** dal centro del perno), stesso hardware, homing, stimatore e
protocollo HIL. Cambia la rete: oltre ai sei ingressi attuali riceve la
**storia dell'ultimo secondo** — i sei ingressi e l'azione dei periodi passati —
e quindi ha **132 ingressi** invece di 6 (132-128-128-1).

Perché l'asta è a 20 cm e non a 30-35: vedi il `LEGGIMI.md` di `Controllo_SwingUp_Asta20`.
**Prima di caricarlo l'asta deve essere a 20 cm dal centro del perno al capo corto.**

## La storia degli ingressi (23/10)

Per ogni ritardo k (in periodi da 20 ms) la rete riceve i sei ingressi di k periodi
fa e l'azione decisa allora (7 numeri), dopo i sei ingressi attuali:

    ritardi = 1, 2, ..., 10  (ogni 20 ms fino a 0,2 s)  e  15, 20, ..., 50  (ogni 100 ms fino a 1 s)
    ingressi = 6 + 7 x 18 = 132

Gli stessi ritardi sono in `config_controllo.h` (`CFG_STORIA_RITARDI`, generato da
`esporta_firmware.py`). Nel firmware (`main.c`, `Storia_*`): un anello di 50 periodi;
dopo ogni inferenza gli ingressi e l'azione di adesso diventano il ritardo 1. A ogni
avvio con B1 (e dopo ogni arresto) la storia riparte riempita con i primi ingressi e
azione 0, esattamente come all'inizio di un episodio in simulazione (`storia.py`).
Lo verificano `tests/test_controllo.py` (ritardi, azioni, riavvio) e `tests/test_hil.py`.

Costo: 33 931 moltiplicazioni per inferenza (il doppio della rete a 6 ingressi), 131,5 KB
di pesi per rete, 1 KB di RAM per le attivazioni. Questo firmware ha una sola rete (con due reti: 307 KB di flash su 512).
Il tempo di inferenza reale si legge in `rete_durata_ms` (deve restare molto sotto i 20 ms).

## Importarlo in STM32CubeIDE

1. *File → Import → General → Existing Projects into Workspace*.
2. *Select root directory*: la cartella `Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap`. Spuntare il
   progetto e premere *Finish* (non selezionare "Copy projects into workspace").
3. *Project → Build Project*: 0 errori. Poi *Run* con la configurazione
   `Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap Debug`.

La cartella `Debug/` non è copiata: la crea CubeIDE al primo Build.
**Non rigenerare il codice da CubeMX**: il `.ioc` descrive ancora la rete X-CUBE-AI a 6
ingressi e la rigenerazione rimetterebbe `X-CUBE-AI/App` a 6 ingressi (il Build si ferma
con `#error ... CFG_STORIA_INGRESSI`). In quel caso rieseguire `esporta_firmware.py` (sotto).

## Uso sul banco

1. Avviare prima il nodo `ponte_mocap` (MOCAP.md). All'accensione la scheda aspetta gli
   angoli (LD2 lampeggia), poi vuole il pendolo fermo appeso per 1 s (riferimento
   iniziale: l'offset porta quell'angolo a 180°), poi homing automatico come prima.
2. Premere **B1**. Il firmware guarda l'angolo arrivato dal tracciamento (senza angoli
   recenti l'avvio è rifiutato con `controllo_errore = 12`):
   - **pendolo appeso** (entro ±20° da -180°, `CFG_ANGOLO_RICALIBRAZIONE_RAD`):
     se l'angolo è rimasto entro 0,5° per almeno 1 s (`CFG_TEMPO_QUIETE_MS`) torna
     esattamente a -180° (nuovo offset) e lo stimatore si azzera, poi la rete fa lo
     **swing-up**. Se il pendolo oscilla ancora l'avvio è rifiutato
     (`controllo_errore = 11`): aspettare che si fermi;
   - **pendolo portato su a mano** (entro ±10° da 0°, `CFG_ANGOLO_MAX_AVVIO_RAD`):
     nessuna ricalibrazione, la rete parte direttamente con l'**equilibrio**;
   - qualunque altro angolo: avvio rifiutato (`controllo_errore = 11`).

   `avvio_modo` nel debugger dice quale partenza è stata accettata
   (1 = swing-up, 2 = equilibrio dall'alto).
3. B1 ferma tutto in qualunque momento.

**Sicurezza**: durante lo swing-up il carrello si muove fino a ±0,40 m dal
centro a 0,48 m/s e l'asta da 0,98 m (0,78 + 0,20) ruota. Tenere libera la zona e la mano
vicino a B1 (o all'alimentazione del driver) nelle prime prove.

## La rete (24/10)

Rete unica (`rete_scelta` = 0): `modelli_asta20_basevecchia_storia1s_20s/migliore.zip` (seed 42,
episodi 20 s, checkpoint a 1,78 M passi su 2,5 M), da
`cart_pendolo_velocita_rl/config_asta20_basevecchia_storia1s_20s.toml`: perno viscoso
log-uniforme 0,0003-0,015 N·m·s, attrito coulombiano 0-0,006 N·m (comprende la stima a memoria
del banco, 0,0095 + 0,006), resistenza dell'aria 0-6,6e-4 N·m·s².

`prova_smorzamento.py`, 20 semi per valore, con disturbi:

| perno | questa rete | rete del 23/10 (`Controllo_SwingUp_Asta20_Storia`, rete 0) |
|---|---:|---:|
| 0,0003-0,005 N·m·s, attrito 0,0005 N·m | 80 / 80 | 11 / 80 |
| 0,0003-0,005 N·m·s, senza attrito coulombiano | 80 / 80 | 3 / 80 |
| stima banco 0,0095 + 0,006 N·m | 16 / 20 (nominale 18 / 20) | 17 / 20 (nominale 18 / 20) |
| 0,01 + attrito 0,0005 | 0 / 20 (arriva su a 13,1 s, oltre i 12 s del criterio) | |

0-0,2 passaggi sopra la verticale; in alto in 5,5-7,5 s (10-11 s con il perno della stima banco).
È l'unica rete del 24/10 che regge sia un perno che frena poco sia la stima del banco.
Dettagli in `cart_pendolo_velocita_rl/TEST_ESEGUITI.md`.

**Prova consigliata**: almeno 3 swing-up; annotare se aggancia, quanto
resta su, `stima.offset` (rad) in equilibrio, `rete_durata_ms` e `controllo_errore`
(4 = limite di corsa, 5 = periodo scaduto, 9 = salto dell'angolo, 11 = avvio rifiutato,
12 = angolo dal tracciamento assente). Con l'encoder ancora montato: confrontare
`theta_deg` e `encoder_theta_deg`.
Confronto utile: stesse prove con `Controllo_SwingUp_Asta20_Storia` (rete del 23/10).

| | Prova_controllo | Controllo_SwingUp / _Asta20 / _Asta20_Storia |
|---|---|---|
| Partenza | pendolo in alto, tenuto a mano | appeso, fermo |
| Costanti di rete/motore/stimatore | `#define` sparse in `main.c` e `stima.h` | `Core/Inc/config_controllo.h`, **generato** da `esporta_firmware.py` |
| Riferimento dell'angolo | solo all'accensione | anche a ogni B1 se il pendolo è fermo e appeso (entro ±20°); dall'alto nessuna ricalibrazione |
| Limite di corsa del controllo | coincide con i finecorsa | centro ± `CFG_LIMITE_CORSA_M` (0,40 m), come in addestramento |
| Lettura dell'angolo impossibile | accettata | arresto, `controllo_errore = 9` (salto > `CFG_SALTO_ENCODER_MAX` conteggi, 25 rad/s, in un periodo; qui sull'angolo del tracciamento) |
| Periodo della rete | 20 ms con deriva di 0–2 ms | `CFG_PERIODO_CONTROLLO_MS` (10 ms dalle reti del 16/10, 20 ms prima), senza deriva |
| Inversione del motore | 2–4 ms fermo, poi ripartenza con salto di frequenza | arresto al fronte STEP, DIR cambiato nell'ISR, rampa ferma a zero per `CFG_TICK_FERMO_INVERSIONE` periodi e poi ripartenza da zero: identica a `stepper.py` (verificato da `tests/test_controllo.py`) |
| Offset dell'encoder (conteggi persi) | non corretto: la rete vede l'angolo sbagliato | stimato come terzo stato dello stimatore (`STIMA_L_B`, solo nella metà alta, limite ±15°): la rete riceve l'angolo vero stimato. Dal 16/10 anche per la rete del 14/10 (`esporta_firmware.py --stimatore-offset -1`) |
| Guasti CPU (HardFault…) | STEP continuava | TIM3 fermato subito |
| Zona morta dell'azione | ignorata | come `stepper.py` |
| `ENCODER_SIGN` | — | -1 (impostato sul banco il 16/10) |
| Ingressi della rete | 6 | 6; qui **132**: i 6 attuali + storia di 1 s (`CFG_STORIA_*`) |

Codici di `controllo_errore` (debugger, Live Expressions): 0 ok, 1 init rete,
2 inferenza, 3 NaN/Inf, 4 finecorsa o limite di corsa, 5 scadenza del periodo,
6 avvio PWM, 7 calibrazione iniziale, **9 salto dell'angolo, 10 angolo oltre il
limite (solo modalità equilibrio), 11 avvio rifiutato (pendolo non fermo),
12 angolo dal tracciamento assente o scaduto (> 50 ms)**.
Altre variabili utili: `angolo_quiete_ms`, `angolo_salti`, `theta_offset_rad`,
`mocap_valido`, `mocap_eta_ms`, `mocap_ricevuti`/`_scartati`/`_non_tracciati`/`_errori_uart`,
`theta_ricalibrazioni`, `avvio_modo`, `limite_controllo_min_steps`/`max_steps`.

Con `swing_up = false` in `config.toml` lo stesso firmware lavora in modalità
solo equilibrio: avvio accettato solo con |angolo| < 10°, arresto oltre 45°.
Le soglie si cambiano in `config.toml` → `[firmware]` e si riesportano con
`esporta_firmware.py` (non serve riaddestrare).

## Aggiornare la rete

Dalla cartella `cart_pendolo_velocita_rl`, dopo un addestramento con `[storia]`:

```sh
python esporta_firmware.py --model modelli_asta20_basevecchia_storia1s_20s/migliore.zip \
    --progetto ../Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap
```

**Serve sempre `--progetto ../Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap`**: senza, lo script
scrive in `Controllo_SwingUp_Asta20_Storia` (`[firmware].nome_progetto` della rete), quello della rete del 23/10.
Scrive `config_controllo.h` (anche `CFG_STORIA_*`), `reti_candidate.c` e i pesi. Se il
numero di ingressi della rete è diverso da quello del codice X-CUBE-AI del progetto
(`AI_NETWORK_IN_1_SIZE` in `X-CUBE-AI/App/network.h`), esporta l'ONNX e **rigenera
`X-CUBE-AI/App`** con `stedgeai` (X-CUBE-AI 10.1.0 in `~/STM32Cube`, stesse opzioni
della generazione del 29/09: `--target stm32f4 --compression none`). Tutte le reti
dello stesso firmware devono avere gli stessi ritardi.

## Test su PC (senza scheda)

Dalla radice del repository, con l'ambiente Python del progetto RL:

```sh
python3 Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap/tests/test_controllo.py   # USER CODE reale con HAL e USART2 simulate, anche la storia
python3 Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap/tests/test_collegamento.py # firmware su PC + nodi ROS veri (rclpy finto), serve pyserial
python3 Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap/tests/test_stima.py       # stima.c = stima.py
python3 Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap/tests/test_hil.py         # protocollo HIL con la rete del firmware (132 ingressi)
python3 Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap/tests/test_candidati.py   # pesi e guadagni della rete = checkpoint
```

`test_stima.py` usa `stima.guadagni_con_offset`, che richiede scipy
(non installato nell'ambiente `cart_pendolo_stepper_rl/.venv`).

La prova HIL sulla scheda vera è identica a quella di `HIL.md`
(`HIL_MODE 1`, poi `./HIL.command --model modelli_asta20_basevecchia_storia1s_20s/migliore.zip`).
La scheda rimanda i sei ingressi attuali; `hil.py` confronta quelli e l'azione
(calcolata con la storia del PC, uguale a quella della scheda).
