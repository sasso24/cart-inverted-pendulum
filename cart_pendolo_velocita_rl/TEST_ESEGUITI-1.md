# Stimatore dell'offset e prove a 10 ms — 16 ottobre 2026

Confronto con `confronta.py`, 40 semi (2000-2039), stesse condizioni per tutte le reti:
*disturbi* = profilo della rete + guasti sensori del 14/10 (offset ±2°, conteggi
persi ≤0,2%); *sensori 16/10* = offset ±3°, ≤0,3%; *offset 4°* = encoder spostato
di 4° fissi; *inversione* = rampa ferma 2 ms al cambio di verso come il firmware;
*aggancio* = partenza da stato casuale, successo se in equilibrio negli ultimi 8 s.

| Rete | nominale | disturbi | sensori 16/10 | offset 4° | inversione | aggancio | inversioni/s (nom.) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 14/10, 20 ms (sulla scheda fino al 16/10) | 40 | 29 | 18 | 0 | 29 | 24 | 22 |
| **14/10 + stimatore dell'offset** (firmware dal 16/10) | **40** | **39** | **37** | **38** | **39** | **40** | 22 |
| 10 ms, seed 7, 2,68 M passi | 40 | 11 | 12 | 2 | 11 | 13 | 15 |
| 10 ms + stimatore dell'offset, seed 42, 0,96 M | 35 | 25 | 26 | 11 | 25 | 40 | 23 |

- Lo stimatore dell'offset da solo, senza riaddestrare, porta la rete del 14/10
  quasi a 40/40 in tutte le condizioni. Test `test_periodo.OffsetTest`: con 4°
  di offset l'offset stimato a fine episodio è entro 1° da quello vero.
- **10 ms non conviene, per ora**: addestramenti a 10 ms (gamma 0,995,
  ricompensa riscalata, 3 M passi) instabili, con crolli della ricompensa
  anche dopo il 100% di successi; le reti migliori restano sotto quella del
  14/10. Prova a 20 ms con sensori 16/10 e partenze casuali ma stimatore a due
  stati: 0% di successi in 1,5 M passi, quindi il problema principale era
  l'offset dell'encoder, non il periodo.
- Firmware `Controllo_SwingUp` (test su PC): `test_controllo.py` OK (rampa C =
  `stepper.py` su 2000 passi casuali), `test_stima.py` OK (C = Python anche con
  offset, differenza < 2e-6), `test_hil.py` 3/3 swing-up, diff. ingressi 9e-7.
  Da compilare in STM32CubeIDE e provare sul banco.

Dati: `risultati/confronto_swingup.json`, `confronto_swingup_offset.json`,
`confronto_10ms.json`, `confronto_offset_960k.json`.

---

# Swing-up da -180° e configurazione unica — 14 ottobre 2026

**Rete** `modelli_swingup/migliore.zip` (6-128-128-1, SAC, seed 42, scelta a
1,02 M passi su 1,5 M; `config.toml` = configurazione salvata con la rete).
Addestramento con disturbi (intensità 0,35, ritardo fino a 4 ms) e guasti
sensori (offset encoder ±2°, conteggi persi fino a 0,2%, 0,5 impulsi spuri/s
da 1-4 conteggi, passi persi fino al 2%), rumore 5-10 mm / 0,5-1°.

**Valutazione** (`valuta.py`, 40 semi 2000-2039 mai usati in addestramento,
episodi da 20 s, partenza appesa ±3°; successo = in equilibrio senza
interruzioni da 12 s alla fine):

| | nominale | disturbi + guasti sensori |
|---|---:|---:|
| Successi | 40/40 | 27/40 |
| In equilibrio alla fine dell'episodio | 40/40 | 28/40 |
| Tempo di salita (medio / max) | 9,4 / 10,2 s | 9,6 / 11,6 s |
| Corsa massima | 0,20 m | 0,37 m (limite 0,40) |
| Arresti per limite di corsa | 0 | 0 |

Stessi 40 semi, rete invariata, un solo tipo di guasto per volta:
senza guasti sensori 40/40; solo offset ±2° 32/40 successi (37/40 in
equilibrio alla fine); guasti senza offset 36/40 (37/40). L'offset iniziale
pesa di più: nel firmware viene azzerato a ogni avvio (ricalibrazione a -180°).

Un controllore euristico di pompaggio dell'energia sullo stesso modello sale
in 8-11 s: con 0,48 m/s di velocità massima del carrello la salita non può
essere molto più rapida. Nelle cadute il carrello deriva verso un lato
(offset/deriva dell'angolo) e la rete spesso fa un giro completo e riprende
il pendolo. Durante l'equilibrio il comando cambia ancora spesso verso
(`peso_inversione = 0,02` non basta).

**Altre prove**

- Codice nuovo con i vecchi checkpoint: `modelli_disturbi/migliore.zip`,
  nominale e disturbato, 1.800 passi: osservazioni, stato e ricompense
  identici bit per bit al codice del 9 ottobre.
- `python -m unittest test_versione test_disturbi test_stima test_configurazione`:
  35 test OK. `verifica.py` fallisce sulla manovra manuale da 10 cm come già
  il codice del 9 ottobre (minimo 3.200 Hz).
- `esporta_onnx.py`: 5.100 ingressi, errore massimo PyTorch/ONNX 5,2e-6.
- Firmware `Controllo_SwingUp`: compilato con arm-none-eabi-gcc 13.2
  (anche `HIL_MODE 1`), 0 warning; text 106.944 B.
  `tests/test_controllo.py` (codice USER CODE reale, HAL simulata: ricalibrazione
  all'avvio, salti encoder, limiti di corsa, B1, fault, rampa) OK;
  `tests/test_stima.py` OK (1,4e-6); `tests/test_hil.py` OK (protocollo e pesi,
  diff ingressi 1,4e-6, diff azione 2,3e-6; swing-up 2/3).
- `Prova_controllo/tests/test_hil.py` e `test_stima.py` passano ancora col
  codice Python nuovo.
- Ricalibrazione condizionata (sera del 14/10): a B1 l'angolo torna a -180°
  solo se quello letto è entro ±20° da -180° (e l'encoder è fermo); entro ±10°
  da 0° la rete parte con l'equilibrio senza ricalibrare; altrove avvio
  rifiutato. `tests/test_controllo.py` OK (anche le soglie), firmware
  ricompilato 0 warning (text 107.072 B). Stessa rete, partenza dall'alto
  ±3° in simulazione: 20/20 in equilibrio per 20 s con sensori ideali,
  16/20 con disturbi e guasti.
- Non provato: scheda reale, motore, encoder sul banco.

---

# Hardware-in-the-loop — 9 ottobre 2026

- Firmware: `HIL_MODE` in `main.c` (default 0). Con 1 la scheda riceve le misure
  simulate via USART2, esegue `stima.c` e la rete X-CUBE-AI, risponde col comando.
  Compilato con arm-none-eabi-gcc in entrambe le modalità, 0 warning.
- PC: `hil.py` / `HIL.command`, istruzioni in `Prova_controllo/HIL.md`.
- Senza scheda (`Prova_controllo/tests/test_hil.py`): codice HIL reale di main.c
  su host, UART su pseudo-terminale, rete come MLP C con i pesi del firmware.
  3/3 episodi stabili, diff ingressi 1,6e-7, diff azione 3,9e-7.
- Da fare sulla scheda: tempi reali, runtime ST, seriale ST-LINK.

# Stimatore nel firmware STM32 — 9 ottobre 2026

- `Prova_controllo/Core/Src/stima.c`: porting di `stima.py`, predizione in
  SysTick ogni 2 ms, correzione in `Rete_Process` ogni 20 ms. Rete aggiornata
  ai pesi di `modelli_disturbi/migliore.zip` (`aggiorna_pesi_xcubeai.py`).
- C contro Python: 20 sequenze da 60 s con wrap ±pi, differenza massima 1,4e-6.
- Catena del firmware in simulazione (stima.c via ctypes + pesi letti da
  `network_data_params.c`, float32): 60/60 episodi da 60 s stabili
  (disturbi 0,25, intensità 1, rumore massimo 10 mm / 1°), angolo max 6,6°.
- `test_controllo.py` aggiornato e superato (ingressi stimati, integrazione
  della velocità STEP, aggancio della posizione a controllo fermo).
- Compilazione e link del firmware completo con arm-none-eabi-gcc 13.2
  (Cortex-M4 hard-float, runtime ST): 0 warning, 97,6 kB di flash.
- Non eseguiti: caricamento sulla scheda, Validate X-CUBE-AI, prova sul banco.
  `tests/test_control_button.py` non compila già nella versione precedente
  (manca il mock della rete X-CUBE-AI).

# SAC dall’alto con rumore dei sensori — 1 ottobre 2026

- Rimosso il controllo LQR (`equilibrio.py`, test, launcher, profilo e documento).
- Rumore: posizione 5–10 mm, encoder 0,5–1° (1 sigma per lettura a 50 Hz,
  estratto per episodio), più quantizzazione 2400 conteggi/giro e offset ±0,125°.
  Disturbi fisici invariati, `intensita = 0.25`.
- Nuovo stimatore `stima.py`: senza di esso anche un controllore ideale
  cadeva in 1–2 s con questi livelli di rumore (velocità derivate inutilizzabili).
- Ricompensa: termine di centratura del carrello più stretto (scala 15 cm)
  e piccola penalità sulle variazioni del comando. Con la vecchia ricompensa
  la rete stava in alto, ma nelle prove da 60 s il carrello derivava fino al finecorsa.
- Addestramento: 400k passi da zero con disturbi, poi 250k passi di rifinitura
  con la nuova ricompensa. Rete scelta: `modelli_disturbi/migliore.zip`.
- `Valuta.command` (semi 2000–2019, 20 s): 20/20 disturbati, 20/20 nominali.
  Rapporto: `risultati/valutazione_disturbi.json`.
- Prova lunga, semi 4000–4019, 60 s per episodio, 100/100 interamente stabili:
  nominale, disturbi 0,25, disturbi a intensità 1, rumore fisso al massimo
  (10 mm, 1°), spinte di ±1 N per 100 ms. Angolo massimo 4,5° (partenza fino
  a 2,9°), carrello entro 15 cm dal centro.
- 25 test automatici superati: `python -m unittest test_versione test_disturbi test_stima`.
- `verifica.py` si ferma nella manovra manuale con le frecce: lo fa anche la
  versione precedente non modificata, quindi non dipende da queste modifiche.
  Prove eseguite con MuJoCo 3.14 (requirements: 3.12).
- Nessuna prova sul banco. Lo stimatore va portato sul firmware (vedi `TODO_STM32.md`).

# Disturbi e vibrazioni - 29 settembre 2026

- 21 test automatici superati: i 14 esistenti e 7 nuovi per configurazioni,
  determinismo, assenza di deriva dei parametri fra reset, modi oscillanti e
  decadimento, integrazione esatta, ritardi, limiti, arresti, separazione fra
  rumore di misura e fisica, compatibilita Gymnasium e contratti checkpoint.
- Il test storico della rampa ora imposta esplicitamente 64.000 Hz/s nella
  propria fixture: assumeva questa rampa anche dopo che il TOML era passato
  a 1.600.000 Hz/s. Nessun cambiamento al parametro hardware.
- Prova SAC temporanea di 3.104 decisioni (incluse ottimizzazioni), valutazione
  doppia nominale/disturbata, salvataggio, ripresa per altre 8 decisioni e
  simulazione senza finestra completati. Non e un addestramento completo.
- La rete esistente `modelli/migliore.zip` e stata valutata sui semi 2000–2019:
  20/20 successi nominali e 20/20 con il profilo `disturbi.toml` a intensita 0,35.
  Ricompense medie rispettivamente 607,98 e 601,90; durata media della migliore
  sequenza stabile 9,287 s e 9,09 s. Nessun finecorsa nelle due suite.
  Rapporto: `risultati/stress_rete_esistente.json`.
- Questi risultati usano il criterio storico di almeno 3 s consecutivi di
  equilibrio. Il profilo iniziale e lieve e non identificato sul banco:
  non dimostra robustezza alle risonanze reali e non riproduce il fallimento
  osservato sull'hardware. Frequenze e ampiezze restano configurabili.
- I checkpoint nominali esistenti superano ancora il controllo di compatibilita.
  XML, ambiente nominale, TOML hardware e firmware non sono stati modificati.
- Nessuna prova hardware o verifica visiva della finestra. La verifica geometrica
  e la manovra manuale storica non sono state rieseguite per questa modifica;
  resta la limitazione della manovra riportata sotto.

# Rampa di frequenza - 28 settembre 2026

- 14 test automatici superati, inclusi salita, frenata, inversione, assenza
  di overshoot, reset, finecorsa e aggiornamento della rampa a ogni passo fisico.
- Verificata la distinzione fra frequenza richiesta e applicata, con stato della
  rampa nel sesto ingresso della rete; Gymnasium e Stable-Baselines3 accettano l'ambiente.
- 1.805 configurazioni geometriche senza collisioni interne alla corsa.
- Prova SAC di 128 decisioni con aggiornamenti della rete, salvataggio,
  caricamento, inferenza e ripresa per altre 16 decisioni riuscita. File di prova
  temporanei; i modelli dell'utente non sono stati modificati.
- Checkpoint precedenti senza rampa rifiutati correttamente.
- La verifica completa `verifica.py` si ferma sulla manovra manuale di 10 cm:
  con il minimo attuale di 3.200 Hz la manovra va in timeout nei due versi.
  Il confronto con il codice precedente alla modifica e gli stessi parametri
  conferma che il problema era gia presente. Con minimo zero, la manovra con
  rampa termina nei due versi entro 0,5 mm dal bersaglio. Il minimo dell'utente
  resta invariato; il rapporto JSON storico non e stato rigenerato.
- Nessuna verifica visiva del viewer, prova hardware o valutazione di una rete
  addestrata con la rampa. Il test breve verifica soltanto la pipeline.

# Verifiche della versione senza rampa — 22 settembre 2026

- Otto test automatici superati: conversioni, limiti, configurazioni invalide,
  microstepping, Gymnasium, accoppiamento del pendolo, arresto e finecorsa,
  episodio completo e reset.
- 1.805 configurazioni geometriche senza collisioni interne alla corsa.
- Manovre manuali di 10 cm completate nei due versi; viewer continuo
  verificato senza aprire la finestra.
- Geometria e opzioni XML confrontate con l'originale: cambia solo l'attuatore.
- Addestramento SAC di prova: 3.200 decisioni, inclusi aggiornamenti della rete,
  selezione del checkpoint, salvataggio e replay buffer.
- Ripresa del checkpoint per altre 64 decisioni riuscita.
- Simulazione headless, valutazione su due semi, generazione CSV e PNG riuscite.
- Vecchi checkpoint a sei ingressi rifiutati correttamente.
- Sintassi dei launcher macOS controllata.

Le prove della pipeline usano Python 3.12 e l'ambiente già presente nel progetto
originale (MuJoCo 3.12.0, Stable-Baselines3 2.9.0, PyTorch 2.14.0).
Gli avvisi Gymnasium sui limiti infiniti delle osservazioni sono attesi.

La rete del test breve non ha imparato lo swing-up: zero successi nelle due
prove. Non viene consegnata come rete addestrata. I checkpoint di test sono
separati nella cartella temporanea `/private/tmp/cart-hz-pipeline-20260922`.
Occorre avviare l'addestramento completo nella nuova cartella e valutarne i risultati.
La finestra interattiva non è stata verificata visivamente; sono stati controllati
il codice del viewer e i suoi comandi manuali tramite simulazione senza GUI.
Nessuna prova hardware eseguita.
