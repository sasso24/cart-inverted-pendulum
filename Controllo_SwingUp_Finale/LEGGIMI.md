# Controllo_SwingUp_Finale — swing-up con asta 0,78 / 0,20 m e storia degli ingressi

Firmware consegnato (24/10): rete addestrata da zero (XML con perno 0,01 N·m·s,
`STIMA_K_D` 0,403, storia di 1 s) con il perno del pendolo **randomizzato a ogni episodio su
un intervallo largo** (vedi "La rete" sotto). Asta **0,78 m / 0,20 m** dal centro del perno.
I progetti firmware precedenti, citati sotto per confronto (`Prova_controllo`,
`Controllo_SwingUp`, `Controllo_SwingUp_Asta20`, `Controllo_SwingUp_Asta20_Storia`, ...), sono
nel ramo `archivio-completo`.

La rete: oltre ai sei ingressi attuali riceve la
**storia dell'ultimo secondo** — i sei ingressi e l'azione dei periodi passati —
e quindi ha **132 ingressi** invece di 6 (132-128-128-1).

Perché l'asta è a 20 cm e non a 30-35: vedi `cart_pendolo_velocita_rl/TEST_ESEGUITI.md`.
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
azione 0, esattamente come all'inizio di un episodio in simulazione (`ConStoria` in `ambiente.py`).
Lo verificano i test su PC `test_controllo.py` (ritardi, azioni, riavvio) e `test_hil.py`.

Costo: 33 931 moltiplicazioni per inferenza (il doppio della rete a 6 ingressi), 131,5 KB
di pesi per rete, 1 KB di RAM per le attivazioni. Questo firmware ha una sola rete (con due reti: 307 KB di flash su 512).
Il tempo di inferenza reale si legge in `rete_durata_ms` (deve restare molto sotto i 20 ms).

## Importarlo in STM32CubeIDE

1. *File → Import → General → Existing Projects into Workspace*.
2. *Select root directory*: la cartella `Controllo_SwingUp_Finale`. Spuntare il
   progetto e premere *Finish* (non selezionare "Copy projects into workspace").
3. *Project → Build Project*: 0 errori. Poi *Run* con la configurazione
   `Controllo_SwingUp_Finale Debug`.

In `Debug/` c'è solo l'eseguibile provato sul banco il 30/10 (`.elf` pronto da caricare);
il resto della build lo ricrea CubeIDE a ogni Build.
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

## La rete (24/10)

Rete unica (`rete_scelta` = 0): `modelli_finale/migliore.zip` (seed 42,
episodi 20 s, checkpoint a 1,78 M passi su 2,5 M), da
`cart_pendolo_velocita_rl/config.toml`: perno viscoso
log-uniforme 0,0003-0,015 N·m·s, attrito coulombiano 0-0,006 N·m (comprende la stima a memoria
del banco, 0,0095 + 0,006), resistenza dell'aria 0-6,6e-4 N·m·s².

`simula.py smorzamento`, 20 semi per valore, con disturbi:

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
(4 = limite di corsa, 5 = periodo scaduto, 9 = salto dell'encoder, 11 = avvio rifiutato).

| | Prima versione (`Prova_controllo`, ramo `archivio-completo`) | Questo firmware |
|---|---|---|
| Partenza | pendolo in alto, tenuto a mano | appeso, fermo |
| Costanti di rete/motore/stimatore | `#define` sparse in `main.c` e `stima.h` | `Core/Inc/config_controllo.h`, **generato** da `esporta_firmware.py` |
| Riferimento dell'angolo | solo all'accensione | anche a ogni B1 se il pendolo è fermo e appeso (entro ±20°); dall'alto nessuna ricalibrazione |
| Limite di corsa del controllo | coincide con i finecorsa | centro ± `CFG_LIMITE_CORSA_M` (0,40 m), come in addestramento |
| Lettura encoder impossibile | accettata | arresto, `controllo_errore = 9` (salto > `CFG_SALTO_ENCODER_MAX` conteggi in un periodo) |
| Periodo della rete | 20 ms con deriva di 0–2 ms | `CFG_PERIODO_CONTROLLO_MS` (10 ms dalle reti del 16/10, 20 ms prima), senza deriva |
| Inversione del motore | 2–4 ms fermo, poi ripartenza con salto di frequenza | arresto al fronte STEP, DIR cambiato nell'ISR, rampa ferma a zero per `CFG_TICK_FERMO_INVERSIONE` periodi e poi ripartenza da zero: identica a `ControlloStepper` in `ambiente.py` (verificato da `test_controllo.py`) |
| Offset dell'encoder (conteggi persi) | non corretto: la rete vede l'angolo sbagliato | stimato come terzo stato dello stimatore (`STIMA_L_B`, solo nella metà alta, limite ±15°): la rete riceve l'angolo vero stimato. Dal 16/10 anche per la rete del 14/10 (`esporta_firmware.py --stimatore-offset -1`) |
| Guasti CPU (HardFault…) | STEP continuava | TIM3 fermato subito |
| Zona morta dell'azione | ignorata | come `ControlloStepper` (`ambiente.py`) |
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
python esporta_firmware.py      # default: modelli_finale/migliore.zip
python esporta_firmware.py --model modelli_riaddestramento/migliore.zip   # dopo un nuovo addestramento
```

Il progetto di destinazione è `[firmware].nome_progetto` della rete (questo progetto);
`--progetto` permette di sceglierne un altro. Scrive `config_controllo.h` (anche `CFG_STORIA_*`), `reti_candidate.c` e i pesi. Se il
numero di ingressi della rete è diverso da quello del codice X-CUBE-AI del progetto
(`AI_NETWORK_IN_1_SIZE` in `X-CUBE-AI/App/network.h`), esporta l'ONNX e **rigenera
`X-CUBE-AI/App`** con `stedgeai` (X-CUBE-AI 10.1.0 in `~/STM32Cube`, stesse opzioni
della generazione del 29/09: `--target stm32f4 --compression none`). Tutte le reti
dello stesso firmware devono avere gli stessi ritardi.

## Test su PC (senza scheda)

I test del firmware sono locali, nella cartella `test/` alla radice (non tracciata da git).
Dalla radice del repository, con l'ambiente Python del progetto RL:

```sh
python test/test_controllo.py   # USER CODE reale con HAL simulata, anche la storia
python test/test_stima_c.py     # stima.c = StimatoreStato (richiede scipy)
python test/test_hil.py         # protocollo HIL con la rete del firmware (132 ingressi)
python test/test_candidati.py   # pesi e guadagni della rete = checkpoint
```

## Prova hardware-in-the-loop (HIL)

La simulazione MuJoCo gira sul Mac; **stimatore (`stima.c`) e rete X-CUBE-AI
girano sulla STM32**. A ogni periodo della rete il Mac manda alla scheda posizione e angolo
simulati (con rumore e disturbi salvati con la rete) e le velocità della rampa;
la scheda risponde con il comando, che pilota la simulazione. In parallelo il
Mac calcola lo stimatore Python e la rete PyTorch e confronta i risultati passo per passo.
La scheda rimanda i sei ingressi attuali; `hil.py` confronta quelli e l'azione
(calcolata con la storia del PC, uguale a quella della scheda).

Verifica: runtime ST e pesi caricati, stimatore in C su Cortex-M4, tempo di
inferenza reale, protocollo. **Non** verifica motore, driver, encoder e meccanica.
In modalità HIL il firmware non fa homing, non legge l'encoder e non comanda mai
il motore. Per sicurezza tieni comunque **spenta l'alimentazione del driver**.

1. In `Core/Src/main.c` scrivere `#define HIL_MODE 1`, *Project → Build Project* (0 errori),
   caricare con *Run*. Il LED verde LD2 resta spento finché il Mac non manda dati, poi
   lampeggia a ogni pacchetto.
2. Chiudere ogni monitor seriale aperto sulla scheda. Dalla cartella `cart_pendolo_velocita_rl`:

   ```sh
   ./Avvia.command hil.py --ping                       # verifica la connessione
   ./Avvia.command hil.py                              # 5 episodi da 20 s
   ./Avvia.command hil.py --episodi 10 --secondi 60    # prova più lunga
   ./Avvia.command hil.py --intensita 1                # disturbi al massimo
   ```

   La porta viene trovata da sola; altrimenti `ls /dev/cu.usbmodem*` e `--porta /dev/cu.usbmodemXXXX`.
   A 115200 baud ogni passo richiede circa 12–15 ms: un episodio da 20 s dura 20–30 s reali.
3. Rimettere `#define HIL_MODE 0`, ricompilare e ricaricare **prima** di riaccendere il driver.

Per ogni episodio viene stampato, per esempio:

```
Episodio 1/5 seed 7000: OK | stabile 20.0 s | angolo max 2.5 deg | diff obs 1.0e-07 | diff azione 2.4e-07 | rete 450 us (max 470)
```

- **OK / CADUTO**: il pendolo, comandato dalla scheda, resta in equilibrio?
- **diff obs**: ingressi della rete calcolati dalla scheda contro lo stimatore Python
  (atteso < 1e-4: differenze di arrotondamento float32).
- **diff azione**: uscita X-CUBE-AI contro PyTorch sugli stessi ingressi
  (atteso < 1e-4; valori grandi = pesi non aggiornati o rete sbagliata).
- **rete**: tempo di `ai_network_run` misurato sulla scheda (molto sotto il periodo di controllo).

I risultati vanno in `cart_pendolo_velocita_rl/risultati/hil/`: `hil_<data>.csv` (ogni passo)
e `hil_<data>.json` (riepilogo).

| Messaggio | Causa probabile |
|---|---|
| `Nessuna scheda trovata` | Cavo solo di alimentazione, o porta diversa: usa `--porta` |
| `Resource busy` / porta occupata | Un monitor seriale (CubeIDE, screen, CoolTerm) è aperto |
| `La scheda non risponde` | Firmware con `HIL_MODE 0`, oppure scheda in pausa nel debugger: premi Resume o il tasto nero RESET |
| `Errore della scheda 2` | Rete X-CUBE-AI non inizializzata o inferenza fallita |
| `diff azione` grande | Pesi in `network_data_params.c` diversi dalla rete `--model` |

Protocollo: USART2 (ST-LINK VCP), 115200 8N1, little-endian. Ogni pacchetto
`A5 5A | corpo | checksum`, con checksum = complemento a due della somma dei byte del corpo.

- PC → scheda, 58 byte: `comando u8` (0 inizio episodio, 1 passo, 2 ping),
  `n u8`, `seq u32`, `v[10] f32` (m/s, una per ogni 2 ms), `x f32` (m),
  `theta f32` (rad), `applied_hz f32`.
- Scheda → PC, 60 byte: `seq u32`, `errore u32`, `azione f32`,
  `richiesta_hz f32`, `osservazione[6] f32`, `stima x, theta, omega f32`,
  `rete_us u32`, `totale_us u32`.

Una richiesta con lo stesso `seq` della precedente non viene rieseguita: la
scheda rimanda la risposta precedente (ritrasmissione dopo un timeout).
