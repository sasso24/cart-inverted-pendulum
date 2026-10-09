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
