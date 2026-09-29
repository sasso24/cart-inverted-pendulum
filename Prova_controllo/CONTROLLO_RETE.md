# Controllo con la rete importata

Dopo il reset vengono inizializzate la rete X-CUBE-AI e le periferiche.
Prima dell'homing, a motore fermo, `Inizializza_Theta()` aspetta che il
conteggio dell'encoder rimanga invariato per 1 secondo, poi fissa theta a
180 gradi (pi radianti). Ogni movimento rilevato fa ripartire questa attesa.
Solo dopo viene eseguito l'homing automatico esistente. Al termine il carrello rimane
fermo al centro. B1 avvia il controllo, una seconda pressione arresta subito
il PWM e spegne LD2. Una nuova pressione riavvia il controllo dalla posizione
corrente, senza ripetere l'homing. L'antirimbalzo resta di 50 ms di rilascio.

Durante l'attesa iniziale lasciare il pendolo libero verso il basso:
l'encoder incrementale puo' verificare che sia fermo, ma non riconoscere
la verticale assoluta. Se non si stabilizza entro 30 secondi l'homing non
parte e `controllo_errore` vale 7; riprovare con un reset. I tempi sono
configurabili con `THETA_STABILE_MS` e `THETA_TIMEOUT_MS` in `main.c`.
Il riferimento acquisito resta valido durante e dopo l'homing; la lettura
alla fine dell'homing aggiorna l'angolo senza azzerarlo. B1 non ricalibra theta.
La convenzione resta zero con il pendolo in alto e `ENCODER_SIGN = -1`. Il verso positivo del carrello e' verso destra
(`DIR_BACKWARD`). La coerenza del verso angolare con la simulazione deve essere
verificata sulla meccanica.

## Ingressi e comando

La rete viene eseguita nel main ogni 20 ms. L'osservazione contiene:

1. Posizione rispetto al centro misurato dall'homing, in metri / 0,462.
2. Seno di theta.
3. Coseno di theta.
4. Velocita' del carrello, stimata dagli STEP degli ultimi 20 ms, / 0,48 m/s.
5. Velocita' angolare, ricavata dall'encoder con gestione del passaggio
   fra -pi e +pi, / 10 rad/s.
6. Frequenza STEP firmata programmata nel timer dopo la rampa / 32000 Hz.

La conversione meccanica e' quella del modello esportato: 200 passi/giro,
32 microstep, cinghia da 2 mm e puleggia da 48 denti, cioe' 15 micrometri/STEP.
La posizione e la velocita' del carrello sono stime degli impulsi comandati:
non rilevano eventuali passi persi. Non si normalizza la posizione dividendo
per la corsa reale: si usa la scala fissa di addestramento, 0,462 m.

L'uscita e' limitata a [-1, 1], senza applicare una seconda tanh. Zero richiede
lo stop; altrimenti la frequenza richiesta e'
`segno(u) * (3200 + abs(u) * 28800)` Hz. SysTick aggiorna la rampa firmata ogni
2 ms, con variazione massima di 3200 Hz, anche in decelerazione e inversione.
La rampa dell'homing resta separata.

Le inversioni e lo zero normale attendono la fine della fase alta di STEP.
Al cambio di DIR il riavvio aspetta il successivo aggiornamento da 2 ms.
Questa pausa e la quantizzazione del timer differiscono dal motore ideale
simulato. Sotto la minima frequenza rappresentabile da TIM3 (circa 16,35 Hz)
il PWM rimane fermo. B1 e gli arresti per errore fermano invece subito il PWM.

## Diagnostica e arresti

Osservabili nel debugger: `rete_osservazione`, `rete_azione`,
`frequenza_richiesta_hz`, `frequenza_applicata_hz`, `velocita_carrello_m_s`,
`velocita_angolare_rad_s`, `rete_inferenze`, `rete_durata_ms`, `rete_pronta`,
`rete_ultimo_errore`, `controllo_errore`, `theta_inizializzato`.

| controllo_errore | Significato |
|---|---|
| 0 | Nessun errore |
| 1 | Inizializzazione X-CUBE-AI fallita; blocco in Error_Handler |
| 2 | Inferenza fallita; rete disabilitata fino al reset |
| 3 | Ingresso o uscita non finiti |
| 4 | Finecorsa premuto o limite della corsa misurata raggiunto |
| 5 | Inferenza oltre 20 ms oppure nessun comando valido per 40 ms |
| 6 | Avvio del PWM fallito |
| 7 | Pendolo non stabile entro il timeout di calibrazione; homing non avviato |

Un errore spegne LD2 e disabilita il controllo, senza ripartenza automatica.
B1 non puo' avviare con un finecorsa premuto, una posizione ai limiti o un
riferimento/rete non valido. Per ripristinare il riferimento dopo un arresto
ai limiti, eseguire il reset e il nuovo homing.

## Verifiche

- Compilazione e link effettuati con GNU Tools for STM32 12.3.rel1,
  Cortex-M4 hard-float, runtime ST `NetworkRuntime1010_CM4_GCC.a`.
- Test del codice C applicativo con HAL e rete simulate:
  `python3 Prova_controllo/tests/test_controllo.py` dalla radice del repository.
  Coprono calibrazione theta prima dell'homing, timeout di stabilita',
  ingressi, uscita, rampa, inversioni, B1 durante l'inferenza,
  antirimbalzo, finecorsa, errori e scadenze temporali.
- Sul Mac corrente il compilatore host richiede un SDK compatibile:
  `SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk python3 Prova_controllo/tests/test_controllo.py`.
- In STM32CubeIDE ricompilare `Prova_controllo`: i percorsi Debug/Release
  ora puntano a questo progetto. I vecchi file Debug erano della copia
  `Lettura_Encoder` e devono essere rigenerati dall'IDE.

Non sono stati eseguiti il caricamento sulla scheda, la misura del tempo di
inferenza sul dispositivo o la prova del pendolo reale. I test host simulano
l'uscita della rete; non validano numericamente il runtime ST o i pesi.
Il link segnala le syscall standard `_read`, `_write`, `_close` e `_lseek`
non implementate (libnosys): il controllo non usa console o file.
