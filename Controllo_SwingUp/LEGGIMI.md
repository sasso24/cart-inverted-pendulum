# Controllo_SwingUp — swing-up e equilibrio con la rete SAC

Copia di `Prova_controllo` (che resta intatto) per la rete addestrata con
`cart_pendolo_velocita_rl/config.toml`: il pendolo parte **appeso (-180°)**,
la rete lo porta su e lo tiene in equilibrio. Stessi pin, timer, homing,
stimatore e protocollo HIL del progetto precedente.

## Importarlo in STM32CubeIDE

1. *File → Import → General → Existing Projects into Workspace*.
2. *Select root directory*: la cartella `Controllo_SwingUp`. Spuntare il
   progetto e premere *Finish* (non selezionare "Copy projects into workspace").
3. *Project → Build Project*: 0 errori. Poi *Run* come per `Prova_controllo`.

Non serve rigenerare il codice da CubeMX. Se lo si rigenera, tutto il codice
nuovo è nei blocchi `USER CODE`, in `stima.c` e in `config_controllo.h`.

## Uso sul banco

1. All'accensione: pendolo libero e fermo appeso (riferimento iniziale), poi
   homing automatico come prima.
2. Premere **B1**. Il firmware guarda l'angolo letto dall'encoder:
   - **pendolo appeso** (entro ±20° da -180°, `CFG_ANGOLO_RICALIBRAZIONE_RAD`):
     se l'encoder è fermo da almeno 1 s (`CFG_TEMPO_QUIETE_MS`) l'angolo torna
     esattamente a -180° e lo stimatore si azzera, poi la rete fa lo
     **swing-up**. Così gli errori accumulati dall'encoder (conteggi persi,
     disturbi) vengono cancellati a ogni avvio. Se il pendolo oscilla ancora
     l'avvio è rifiutato (`controllo_errore = 11`): aspettare che si fermi;
   - **pendolo portato su a mano** (entro ±10° da 0°, `CFG_ANGOLO_MAX_AVVIO_RAD`):
     nessuna ricalibrazione, la rete parte direttamente con l'**equilibrio**
     (anche se la mano trema un po'). L'angolo è quello contato dall'encoder
     dall'accensione: va portato su solo dopo homing e calibrazione iniziale;
   - qualunque altro angolo: avvio rifiutato (`controllo_errore = 11`).

   `avvio_modo` nel debugger dice quale partenza è stata accettata
   (1 = swing-up, 2 = equilibrio dall'alto).
3. B1 ferma tutto in qualunque momento.

**Sicurezza**: durante lo swing-up il carrello si muove fino a ±0,40 m dal
centro a 0,48 m/s e l'asta da 0,82 m ruota. Tenere libera la zona e la mano
vicino a B1 (o all'alimentazione del driver) nelle prime prove.

## Cosa cambia rispetto a Prova_controllo

| | Prova_controllo | Controllo_SwingUp |
|---|---|---|
| Partenza | pendolo in alto, tenuto a mano | appeso, fermo |
| Costanti di rete/motore/stimatore | `#define` sparse in `main.c` e `stima.h` | `Core/Inc/config_controllo.h`, **generato** da `esporta_firmware.py` |
| Riferimento dell'angolo | solo all'accensione | anche a ogni B1 se il pendolo è fermo e appeso (entro ±20°); dall'alto nessuna ricalibrazione |
| Limite di corsa del controllo | coincide con i finecorsa | centro ± `CFG_LIMITE_CORSA_M` (0,40 m), come in addestramento |
| Lettura encoder impossibile | accettata | arresto, `controllo_errore = 9` (salto > `CFG_SALTO_ENCODER_MAX` conteggi in 20 ms) |
| Periodo di 20 ms | deriva di 0–2 ms | fisso, senza deriva |
| Guasti CPU (HardFault…) | STEP continuava | TIM3 fermato subito |
| Zona morta dell'azione | ignorata | come `stepper.py` |
| `ENCODER_SIGN` | — | 1 (il valore impostato il 14/10 in `Prova_controllo`) |

Codici di `controllo_errore` (debugger, Live Expressions): 0 ok, 1 init rete,
2 inferenza, 3 NaN/Inf, 4 finecorsa o limite di corsa, 5 scadenza dei 20 ms,
6 avvio PWM, 7 calibrazione iniziale, **9 salto encoder, 10 angolo oltre il
limite (solo modalità equilibrio), 11 avvio rifiutato (pendolo non fermo)**.
Altre variabili utili: `encoder_quiete_ms`, `encoder_salti`,
`theta_ricalibrazioni`, `avvio_modo`, `limite_controllo_min_steps`/`max_steps`.

Con `swing_up = false` in `config.toml` lo stesso firmware lavora in modalità
solo equilibrio: avvio accettato solo con |angolo| < 10°, arresto oltre 45°.
Le soglie si cambiano in `config.toml` → `[firmware]` e si riesportano con
`esporta_firmware.py` (non serve riaddestrare).

## Aggiornare la rete

Dalla cartella `cart_pendolo_velocita_rl`, dopo un addestramento:

```sh
python esporta_firmware.py --model modelli_swingup/migliore.zip
```

Scrive `config_controllo.h` e i pesi in `X-CUBE-AI/App/network_data_params.c`
usando la configurazione **salvata con la rete**; poi Build e Run.
Funziona finché la rete è 6-128-128-1 (`strati = [128, 128]`); con altre
taglie va reimportato l'ONNX in CubeMX (X-CUBE-AI).

## Test su PC (senza scheda)

Dalla radice del repository, con l'ambiente Python del progetto RL:

```sh
python3 Controllo_SwingUp/tests/test_controllo.py   # codice USER CODE reale con HAL simulata
python3 Controllo_SwingUp/tests/test_stima.py       # stima.c = stima.py
python3 Controllo_SwingUp/tests/test_hil.py         # protocollo HIL con la rete del firmware
```

La prova HIL sulla scheda vera è identica a quella di `HIL.md`
(`HIL_MODE 1`, poi `./HIL.command --model modelli_swingup/migliore.zip`).
