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
