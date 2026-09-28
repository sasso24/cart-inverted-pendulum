# Contratto previsto per STM32

Non è firmware pronto e nessuno script di questo progetto comanda hardware.

- Preparare i sei ingressi nell'ordine e nelle scale di LEGGIMI.md, inclusa
  la frequenza applicata dalla rampa normalizzata alla frequenza massima.
- Eseguire l'attore deterministico SAC (media seguita da tanh) a 50 Hz.
- Convertire l'uscita [-1,1] in Hz firmati con la stessa configurazione e
  la stessa gestione di minimo/zona morta di `stepper.py`.
- Applicare alla richiesta la rampa simmetrica `rampa_hz_s` a 500 Hz (dt=2 ms),
  con lo stesso algoritmo di `ControlloStepper.avanza`. La richiesta resta
  costante tra due decisioni della rete. Zero richiede una frenata a rampa;
  nelle inversioni la frequenza firmata attraversa lo zero. Durante i transitori
  sono ammesse frequenze inferiori a `frequenza_min_hz`.
- Il timer genera STEP a `abs(f_hz)`; DIR dipende dal segno e dal cablaggio.
  A zero fermare gli impulsi, senza dividere per zero. Rispettare i tempi DIR
  e gli impulsi minimi del driver. La risoluzione del timer introduce arrotondamento.
- La frequenza è impulsi/secondo, non giri/secondo. Non integrare più l'azione
  come accelerazione. La rampa firmware deve coincidere con quella simulata.
- Implementare homing, conteggio degli impulsi emessi, acquisizione angolare,
  gestione del wrap e stima delle velocità. I sensori ideali di MuJoCo non sono
  automaticamente equivalenti alle stime disponibili sul banco.
- Finecorsa, timeout e arresto devono funzionare indipendentemente dalla rete.
  Gli arresti di sicurezza azzerano anche lo stato della rampa, senza decelerazione.
- Adattare eventuali esportatori ONNX/C alla rete 6→128→128→1 con uscita
  velocità normalizzata; verificare PyTorch/C su vettori identici. I vecchi
  esportatori con uscita forza/accelerazione non sono intercambiabili.
- Confermare i parametri provvisori di `stepper.toml` e verificare sul banco
  l'intervallo di frequenze sostenibile. Il servo di velocità MuJoCo non ha
  un limite di coppia e non modella passi persi.
