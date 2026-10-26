# Controllo_SwingUp_Asta20_Perno — asta 0,78 / 0,20 m, reti del 24/10 (perno randomizzato)

Copia di `Controllo_SwingUp_Asta20` (24/10) con le **reti nuove**, addestrate con il perno
del pendolo randomizzato (vedi "Le reti" sotto). `Controllo_SwingUp_Asta20` resta con le
reti del 21/10, per confrontarle sul banco (anche con un altro encoder).
Stesso codice di `Controllo_SwingUp_Asta20`; cambiano `config_controllo.h` (`STIMA_K_D` e
guadagni dello stimatore), `reti_candidate.c` e i pesi in `X-CUBE-AI/App`.

Origine: copia di `Controllo_SwingUp` con **la stessa asta da 0,98 m
spostata nel mozzo**: braccio lungo **0,78 m** (prima 0,82), braccio corto **0,20 m**
(prima 0,16), misurati dal centro del perno. Stesso codice, pin, timer, homing,
stimatore e protocollo HIL; cambiano solo `config_controllo.h` (coefficienti
dello stimatore di questa asta) e le reti, addestrate su
`cart_pendolo_velocita_rl/cart_pendolo_asta20_perno.xml` (dal 24/10: perno del pendolo
randomizzato, vedi sotto).

**Prima di caricarlo spostare l'asta**: 20 cm dal centro del perno al capo corto.
Le reti del progetto `Controllo_SwingUp` vanno usate solo con l'asta 0,82 / 0,16.

## Perché 20 cm e non 30-35 cm (21/10)

Con il braccio corto più lungo il baricentro si avvicina al perno: forza del pendolo
sul carrello e inerzia calano. Però, con lo smorzamento del perno del modello
(0,01 N·m·s, valore **stimato, mai misurato**) e il carrello a 0,48 m/s, **dai
~22 cm in su il carrello non riesce più a dare abbastanza energia per lo swing-up**.
Simulazione del pompaggio migliore trovato, energia massima (serve almeno 1 per arrivare in alto):

| braccio corto | 0,16 | 0,18 | **0,20** | 0,22 | 0,24 | 0,30 | 0,33 | 0,35 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| energia massima | 10,7 | 1,29 | **1,20** | 1,08 | 0,94 | 0,30* | 0,02* | −0,17* |

(*pompaggio bang-bang semplice). Le reti addestrate a 0,30 / 0,33 / 0,35 m (9 reti,
~1 M passi ciascuna) restano tutte bloccate a ~110° dal basso. Con 0,20 m il
margine sull'energia è del 20%; rispetto a 0,16 m la forza di reazione sul
carrello cala del 12% e l'inerzia del 13%, la caduta in alto ha la stessa velocità
(k_g 17,3 contro 17,1 1/s²).

**Per arrivare a 30-35 cm** serve una delle due:
- smorzamento reale del perno ≤ ~0,0025 N·m·s per 0,33 m (≤ 0,005 per 0,30 m). Prova al banco:
  pendolo appeso, motore spento, spostarlo di ~20° e lasciarlo; misurare il tempo
  `t½` in cui l'ampiezza si dimezza. Smorzamento = 1,386 / t½ × inerzia (inerzia
  0,0160 kg·m² con l'asta a 0,33 m; 0,0248 con 0,20 m). Poi `damping` nell'XML e
  nuovo addestramento;
- carrello più veloce: con 0,72 m/s anche 0,30 m arriva a energia 1,10.

## Importarlo in STM32CubeIDE

1. *File → Import → General → Existing Projects into Workspace*.
2. *Select root directory*: la cartella `Controllo_SwingUp_Asta20_Perno`. Spuntare il
   progetto e premere *Finish* (non selezionare "Copy projects into workspace").
3. *Project → Build Project*: 0 errori. Poi *Run* con la configurazione
   `Controllo_SwingUp_Asta20_Perno Debug`.

La cartella `Debug/` non è copiata: la crea CubeIDE al primo Build.
Non serve rigenerare il codice da CubeMX. Se lo si rigenera, tutto il codice
nuovo è nei blocchi `USER CODE`, in `stima.c` e in `config_controllo.h`.

## Uso sul banco

1. All'accensione: pendolo libero e fermo appeso (riferimento iniziale), poi
   homing automatico come prima.
2. Premere **B1**. Il firmware guarda l'angolo letto dall'encoder:
   - **pendolo appeso** (entro ±20° da -180°, `CFG_ANGOLO_RICALIBRAZIONE_RAD`):
     se l'encoder è fermo da almeno 1 s (`CFG_TEMPO_QUIETE_MS`) l'angolo torna
     esattamente a -180° e lo stimatore si azzera, poi la rete fa lo
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

## Le reti (24/10)

**Perché nuove reti**: sul banco lo swing-up riusciva ma il pendolo passava sopra la
verticale e continuava a girare. Le reti del 21/10 erano addestrate con lo smorzamento
del perno del modello (0,01 N·m·s, mai misurato): non passavano mai sopra la verticale
e non avevano imparato a frenare. Con un perno che frena meno (0,0025) agganciavano 0-2
volte su 20. Le nuove reti sono addestrate con il perno **randomizzato a ogni episodio**
(viscoso 0,0003-0,012 N·m·s, coulombiano 0-0,003 N·m, resistenza dell'aria) e lo
stimatore usa `STIMA_K_D` 0,0807 (= 0,002 N·m·s, centro dell'intervallo) invece di 0,403.

Asta 0,78 / 0,20 m, stimatore dell'offset anche in addestramento, sensori del 16/10,
20 ms, episodi da 20 s, 2 M passi. Scelte tra 3 addestramenti
(`cart_pendolo_velocita_rl/config_asta20_perno_20s*.toml`, seed 42, 7, 3).

| `rete_scelta` | Rete | perno 0,0003-0,005 N·m·s, disturbi (senza / con attrito coulombiano) | `confronta.py` 20 s: successo (su e mai caduto) su 240 | inversioni/s (nom. / dist.) |
|---|---|---:|---:|---:|
| **1** (all'accensione) | `modelli_asta20_perno_20s_s3` (seed 3) | 77 / 80, 80 / 80 | 216 (236) | 10,6 / 7,0 |
| **2** | `modelli_asta20_perno_20s_s7` (seed 7) | 58 / 80, 78 / 80 | 205 (233) | 28,6 / 7,1 |
| 0 | copia della rete 1 (la rete 0 deve esistere: è quella nel codice X-CUBE-AI) | | | |
| rif. | `modelli_asta20_20s` (21/10, prima rete 0) | 4 / 80 | | |

(`prova_smorzamento.py`, 20 semi per valore di smorzamento; dettagli in
`cart_pendolo_velocita_rl/TEST_ESEGUITI.md`.) Il pendolo arriva in alto in 6-7,5 s e
non lo supera quasi mai (rete 1: 0,1-1,6 passaggi sopra la verticale in 20 s; reti del 21/10: 5-12). **Limite**: se il perno
reale fosse davvero a 0,01 N·m·s, con i disturbi le nuove reti spesso non salgono in 20 s.

**Cambiare rete al banco**: a controllo fermo (LED spento), in *Live Expressions*
scrivere `rete_scelta` = 1 o 2. Il cambio è immediato: `rete_attiva` diventa
uguale a `rete_scelta` (se resta diverso, il controllo era attivo: fermare con B1).
Poi B1 come sempre. `controllo_errore = 1` dopo un cambio = inizializzazione
della rete fallita.

**Prova consigliata**: rete 1, almeno 3 swing-up; annotare se aggancia, quanto
resta su, `stima.offset` (rad) in equilibrio e `controllo_errore`
(4 = limite di corsa, 9 = salto dell'encoder, 11 = avvio rifiutato). Poi rete 2.
Prima delle prove conviene misurare lo smorzamento del perno (sopra): se il
pendolo oscilla a lungo da appeso il perno frena poco, come previsto dalle nuove reti.
Se lo swing-up si ferma prima dell'alto (oscilla sempre più ampio ma non arriva),
il perno reale frena più del modello: fare la prova dello smorzamento sopra.


| | Prova_controllo | Controllo_SwingUp / _Asta20 |
|---|---|---|
| Partenza | pendolo in alto, tenuto a mano | appeso, fermo |
| Costanti di rete/motore/stimatore | `#define` sparse in `main.c` e `stima.h` | `Core/Inc/config_controllo.h`, **generato** da `esporta_firmware.py` |
| Riferimento dell'angolo | solo all'accensione | anche a ogni B1 se il pendolo è fermo e appeso (entro ±20°); dall'alto nessuna ricalibrazione |
| Limite di corsa del controllo | coincide con i finecorsa | centro ± `CFG_LIMITE_CORSA_M` (0,40 m), come in addestramento |
| Lettura encoder impossibile | accettata | arresto, `controllo_errore = 9` (salto > `CFG_SALTO_ENCODER_MAX` conteggi in un periodo) |
| Periodo della rete | 20 ms con deriva di 0–2 ms | `CFG_PERIODO_CONTROLLO_MS` (10 ms dalle reti del 16/10, 20 ms prima), senza deriva |
| Inversione del motore | 2–4 ms fermo, poi ripartenza con salto di frequenza | arresto al fronte STEP, DIR cambiato nell'ISR, rampa ferma a zero per `CFG_TICK_FERMO_INVERSIONE` periodi e poi ripartenza da zero: identica a `stepper.py` (verificato da `tests/test_controllo.py`) |
| Offset dell'encoder (conteggi persi) | non corretto: la rete vede l'angolo sbagliato | stimato come terzo stato dello stimatore (`STIMA_L_B`, solo nella metà alta, limite ±15°): la rete riceve l'angolo vero stimato. Dal 16/10 anche per la rete del 14/10 (`esporta_firmware.py --stimatore-offset -1`) |
| Guasti CPU (HardFault…) | STEP continuava | TIM3 fermato subito |
| Zona morta dell'azione | ignorata | come `stepper.py` |
| `ENCODER_SIGN` | — | -1 (impostato sul banco il 16/10) |

Codici di `controllo_errore` (debugger, Live Expressions): 0 ok, 1 init rete,
2 inferenza, 3 NaN/Inf, 4 finecorsa o limite di corsa, 5 scadenza del periodo,
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
python esporta_firmware.py --model modelli_asta20_perno_20s_s3/migliore.zip \
    --candidato modelli_asta20_perno_20s_s3/migliore.zip --candidato modelli_asta20_perno_20s_s7/migliore.zip \
    --progetto ../Controllo_SwingUp_Asta20_Perno
```

**Serve sempre `--progetto ../Controllo_SwingUp_Asta20_Perno`**: senza, lo script scrive nel
progetto di `[firmware].nome_progetto` della rete, che per le reti `modelli_asta20_perno_*`
è `Controllo_SwingUp_Asta20` (quello con le reti del 21/10).

Scrive `config_controllo.h` e i pesi in `X-CUBE-AI/App/network_data_params.c`
usando la configurazione **salvata con la rete**; poi Build e Run.
Funziona finché la rete è 6-128-128-1 (`strati = [128, 128]`); con altre
taglie va reimportato l'ONNX in CubeMX (X-CUBE-AI).

## Test su PC (senza scheda)

Dalla radice del repository, con l'ambiente Python del progetto RL:

```sh
python3 Controllo_SwingUp_Asta20_Perno/tests/test_controllo.py   # codice USER CODE reale con HAL simulata
python3 Controllo_SwingUp_Asta20_Perno/tests/test_stima.py       # stima.c = stima.py
python3 Controllo_SwingUp_Asta20_Perno/tests/test_hil.py         # protocollo HIL con la rete del firmware
python3 Controllo_SwingUp_Asta20_Perno/tests/test_candidati.py   # pesi e guadagni delle 3 reti = checkpoint
```

`test_stima.py` usa `stima.guadagni_con_offset`, che richiede scipy
(non installato nell'ambiente `cart_pendolo_stepper_rl/.venv` al 21/10).

La prova HIL sulla scheda vera è identica a quella di `HIL.md`
(`HIL_MODE 1`, poi `./HIL.command --model modelli_asta20_20s/migliore.zip`).
