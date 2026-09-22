# Interfaccia futura — NUCLEO-F446RE

Questo file descrive le parti da implementare. Non contiene firmware pronto.

1. Identificare driver e segnali: STEP, DIR, ENABLE, polarità e tempi minimi;
   verificare livelli elettrici e assegnare pin/timer in CubeMX.
2. Eseguire homing lento con arretramento e secondo avvicinamento; inizializzare
   il conteggio e il centro della corsa. Gestire timeout e finecorsa già premuto.
3. Generare STEP con timer (eventualmente DMA) e contare gli impulsi realmente
   emessi, con segno. Gestire inversione DIR e overflow del contatore.
4. Acquisire l'angolo: zero = braccio lungo in alto, verso coerente con MuJoCo.
   Ricavare omega gestendo il wrap ±π e un filtro con ritardo misurato.
5. Ogni 20 ms preparare i sei ingressi nell'ordine del README, eseguire l'attore
   deterministico e limitare l'uscita a [-1,1]. Controllare valori non finiti.
6. Moltiplicare per accelerazione_max, integrare con il tempo effettivo e limitare
   la velocità. Generare la traiettoria a frequenza sufficiente: non basta inviare
   un blocco di impulsi ogni 20 ms. Preservare la frazione di impulso.
7. Portare finecorsa, timeout e disabilitazione driver fuori dalla dipendenza
   dall'inferenza. Definire la frenata e il comportamento dopo l'arresto.
8. Adattare l'esportazione alla rete 6→128→128→1 e all'uscita accelerazione:
   confrontare PyTorch / ONNX / C su vettori di prova e misurare RAM, Flash e tempo
   sulla F446RE. I vecchi wrapper forza non sono compatibili.
9. Confrontare traiettorie reali e simulate prima di provare lo swing-up.

Il PD in stepper.py è una rappresentazione della fisica per MuJoCo. NON copiarlo
nel firmware senza un sensore di posizione: usa x e v reali, non disponibili
contando soltanto gli impulsi. Anche la soglia di errore di inseguimento è una
verifica di simulazione, non una protezione implementabile senza misura.
