# Verifiche della nuova versione — 22 settembre 2026

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
