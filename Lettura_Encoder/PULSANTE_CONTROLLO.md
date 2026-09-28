# Pulsante blu B1: abilitazione del movimento

Al reset `Inizializza()` esegue automaticamente l'homing come prima,
prima di entrare nel ciclo principale. B1 viene abilitato solo dopo
l'homing riuscito: durante l'inizializzazione non arresta il motore e le
pressioni non vengono accodate. Gli errori di homing passano a
`Error_Handler()` come in precedenza.

Nel ciclo principale il controllo parte disabilitato, con LD2 spento:

- Prima pressione dopo il rilascio: abilita i movimenti e accende LD2.
- Pressione durante il controllo: ferma il PWM STEP direttamente
  nell'interrupt, disabilita i movimenti e spegne LD2.
- Pressione successiva: riabilita il ciclo principale senza ripetere
  l'homing. La sequenza di prova ricomincia dall'inizio.

La variabile `controllo_abilitato` è visibile nel debugger. Il firmware
attuale contiene una sequenza di prova, non ancora la policy di controllo.
Il pulsante conserva il riferimento ottenuto all'avvio; la posizione
resta una stima degli impulsi comandati e uno stop a metà impulso può
introdurre un errore nel conteggio.

## Interrupt e antirimbalzo

B1 su PC13 usa EXTI15_10 con priorità di preemption 0, TIM3 usa 1 e SysTick
usa 15; il raggruppamento è NVIC_PRIORITYGROUP_4. B1 può quindi interrompere
il callback del motore. La configurazione è riportata anche nel file .ioc.
L'IRQ viene abilitato dal main solo dopo il completamento di `Inizializza()`.

Lo stop avviene al primo fronte di pressione accettato, senza attese nella
ISR. I rimbalzi non riattivano il controllo: prima di una nuova pressione
occorrono almeno 50 ms di rilascio stabile, verificati da SysTick; entrambi
i fronti EXTI azzerano il tempo di rilascio. Una richiesta di riavvio viene
consumata solo dopo l'uscita dalla precedente sequenza.

Brevi sezioni critiche senza attese proteggono il controllo seguito
dall'avvio PWM, gli aggiornamenti condivisi e il riarmo del pulsante.
Un EXTI arrivato in queste sezioni viene servito al ripristino degli
interrupt. La priorità 0 è la massima configurabile: non precede NMI,
fault a priorità fissa o interrupt globalmente mascherati. La latenza
massima reale non è stata misurata.

## Verifiche

Build ARM completata senza warning. I test host compilano il codice
applicativo estratto da main.c con GPIO e timer simulati:

```sh
python3 Lettura_Encoder/tests/test_control_button.py
```

Coprono homing automatico indipendente da B1, pressioni ignorate nelle
sue tre fasi, controllo inizialmente disabilitato nel main, toggle senza
ripetere l'homing, rimbalzi, richieste di avvio annullate/in coda, stop
prima e durante l'avvio PWM, attese interrompibili e overflow del tick.

Su questo Mac il linker richiede un SDK compatibile:

```sh
SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk python3 Lettura_Encoder/tests/test_control_button.py
```

Resta da verificare sulla scheda: homing automatico al reset, attesa di B1
all'ingresso nel ciclo principale, stop durante il controllo, riattivazione
senza nuovo homing, assenza di riavvii con rimbalzi e latenza tra PC13 e
l'ultimo impulso STEP con analizzatore logico.

Questo comando interrompe gli impulsi STEP; non disalimenta il driver né
realizza un arresto di emergenza hardware indipendente dal firmware.
