# Controllo_SwingUp_Asta20_Storia — swing-up con asta 0,78 / 0,20 m e storia degli ingressi

Copia di `Controllo_SwingUp_Asta20` (che resta intatto): stessa asta spostata
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
di pesi per rete, 1 KB di RAM per le attivazioni. Firmware con due reti: 307 KB di flash su 512.
Il tempo di inferenza reale si legge in `rete_durata_ms` (deve restare molto sotto i 20 ms).

## Importarlo in STM32CubeIDE

1. *File → Import → General → Existing Projects into Workspace*.
2. *Select root directory*: la cartella `Controllo_SwingUp_Asta20_Storia`. Spuntare il
   progetto e premere *Finish* (non selezionare "Copy projects into workspace").
3. *Project → Build Project*: 0 errori. Poi *Run* con la configurazione
   `Controllo_SwingUp_Asta20_Storia Debug`.

La cartella `Debug/` non è copiata: la crea CubeIDE al primo Build.
**Non rigenerare il codice da CubeMX**: il `.ioc` descrive ancora la rete X-CUBE-AI a 6
ingressi e la rigenerazione rimetterebbe `X-CUBE-AI/App` a 6 ingressi (il Build si ferma
con `#error ... CFG_STORIA_INGRESSI`). In quel caso rieseguire `esporta_firmware.py` (sotto).

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

## Le due reti

Asta 0,78 / 0,20 m, storia di 1 s, episodi da 20 s, stimatore dell'offset anche in
addestramento, sensori del 16/10, 2 M passi. Scelte tra 4 addestramenti
(`cart_pendolo_velocita_rl/config_asta20_storia*.toml`: storia di 1 s o di 0,2 s, seed 42 e 7).

Simulazione `confronta.py --durata 60`, 40 semi mai visti per condizione;
"successo / su" = in equilibrio da 12 s alla fine / arrivato su entro 20 s e mai caduto:

| `rete_scelta` | Rete | nominale | disturbi | sensori 16/10 | offset 4° | aggancio | totale | salita (mediana / max, disturbi) | inversioni/s (nom. / dist.) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| **0** (all'accensione) | `modelli_asta20_storia1s_20s` (seed 42) | 40 / 40 | 40 / 40 | 39 / 40 | 31 / 40 | 40 / 40 | 230 / 240 | 10,0 / 11,4 s | 16,8 / 11,4 |
| **1** | `modelli_asta20_storia1s_20s_s7` (seed 7) | 40 / 40 | 39 / 40 | 39 / 40 | 31 / 39 | 40 / 40 | 228 / 239 | 10,1 / 12,2 s | 35,2 / 10,3 |
| — | storia 0,2 s, seed 7 (non nel firmware) | 40 / 40 | 40 / 40 | 37 / 40 | 34 / 40 | 40 / 40 | 231 / 240 | 9,8 / 11,6 s | 40,8 / 12,8 |
| — | storia 0,2 s, seed 42 (non nel firmware) | 40 / 40 | 37 / 40 | 38 / 40 | 27 / 40 | 40 / 40 | 219 / 240 | 9,5 / 14,6 s | 21,8 / 10,5 |
| rif. | asta 0,20 m senza storia (`Controllo_SwingUp_Asta20`, rete 0) | 38 / 40 | 31 / 40 | 27 / 40 | 18 / 40 | 40 / 40 | 185 / 240 | 11,2 / 12,7 s | 11,5 / 7,6 |
| rif. | asta 0,16 m senza storia (`modelli_60s_1200k`) | 40 / 40 | 39 / 40 | 40 / 40 | 28 / 38 | 40 / 40 | 226 / 238 | 9,9 / 12,6 s | 19,8 / 8,0 |

(totale = somma delle sei condizioni, inclusa *inversione*, uguale a *disturbi*.)

Con la storia le reti a 0,20 m passano da 185 a 228-230 successi su 240, salgono
~1 s prima e, una volta su, non cadono. La rete 0 è scelta al posto di quella da 0,2 s
(seed 7, 231) perché cambia verso al motore meno della metà delle volte (16,8 contro
40,8 inversioni/s); le reti da 0,2 s hanno 76 ingressi e non possono stare nello stesso
firmware di quelle da 1 s.

**Cambiare rete al banco**: a controllo fermo (LED spento), in *Live Expressions*
scrivere `rete_scelta` = 0 o 1. `rete_attiva` lo conferma (se resta diverso, il
controllo era attivo: fermare con B1). `controllo_errore = 1` dopo un cambio =
inizializzazione della rete fallita.

**Prova consigliata**: rete 0, almeno 3 swing-up; annotare se aggancia, quanto
resta su, `stima.offset` (rad) in equilibrio, `rete_durata_ms` e `controllo_errore`
(4 = limite di corsa, 5 = periodo scaduto, 9 = salto dell'encoder, 11 = avvio rifiutato).
Poi rete 1. Confronto utile: stesse prove con `Controllo_SwingUp_Asta20` (senza storia).

| | Prova_controllo | Controllo_SwingUp / _Asta20 / _Asta20_Storia |
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
| Ingressi della rete | 6 | 6; qui **132**: i 6 attuali + storia di 1 s (`CFG_STORIA_*`) |

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

Dalla cartella `cart_pendolo_velocita_rl`, dopo un addestramento con `[storia]`:

```sh
python esporta_firmware.py --model modelli_asta20_storia1s_20s/migliore.zip \
    --candidato modelli_asta20_storia1s_20s_s7/migliore.zip \
    --progetto ../Controllo_SwingUp_Asta20_Storia
```

Scrive `config_controllo.h` (anche `CFG_STORIA_*`), `reti_candidate.c` e i pesi. Se il
numero di ingressi della rete è diverso da quello del codice X-CUBE-AI del progetto
(`AI_NETWORK_IN_1_SIZE` in `X-CUBE-AI/App/network.h`), esporta l'ONNX e **rigenera
`X-CUBE-AI/App`** con `stedgeai` (X-CUBE-AI 10.1.0 in `~/STM32Cube`, stesse opzioni
della generazione del 29/09: `--target stm32f4 --compression none`). Tutte le reti
dello stesso firmware devono avere gli stessi ritardi.

## Test su PC (senza scheda)

Dalla radice del repository, con l'ambiente Python del progetto RL:

```sh
python3 Controllo_SwingUp_Asta20_Storia/tests/test_controllo.py   # USER CODE reale con HAL simulata, anche la storia
python3 Controllo_SwingUp_Asta20_Storia/tests/test_stima.py       # stima.c = stima.py
python3 Controllo_SwingUp_Asta20_Storia/tests/test_hil.py         # protocollo HIL con la rete del firmware (132 ingressi)
python3 Controllo_SwingUp_Asta20_Storia/tests/test_candidati.py   # pesi e guadagni delle 2 reti = checkpoint
```

`test_stima.py` usa `stima.guadagni_con_offset`, che richiede scipy
(non installato nell'ambiente `cart_pendolo_stepper_rl/.venv`).

La prova HIL sulla scheda vera è identica a quella di `HIL.md`
(`HIL_MODE 1`, poi `./HIL.command --model modelli_asta20_storia1s_20s/migliore.zip`).
La scheda rimanda i sei ingressi attuali; `hil.py` confronta quelli e l'azione
(calcolata con la storia del PC, uguale a quella della scheda).
