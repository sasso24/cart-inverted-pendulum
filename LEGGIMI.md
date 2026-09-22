# Pendolo su cinghia dentata — nuova versione SAC

Questo progetto è separato da `cart_pendolo_rl`: non ne modifica codice, XML,
checkpoint o esportazioni STM32. Riproduce la **configurazione meccanica** della
foto, con dimensioni e parametri provvisori: non è ancora un modello identificato
del banco reale. Non contiene codice che aziona hardware.

## Avvio rapido

Sul computer attuale puoi usare subito i launcher: se manca una `.venv` locale,
riutilizzano Python e le librerie di `../cart_pendolo_rl/.venv`, senza modificarli.

- `Avvia_RL.command`: mostra la migliore policy di questo progetto.
- `Addestra.command`: nuovo addestramento da zero di 300.000 decisioni; protegge un
  addestramento già presente. Per riprendere usa il comando sotto.
- `Valuta.command`: 20 episodi su semi diversi da quelli di selezione.
- `Verifica.command`: controlla ambiente, meccanica, conteggio e arresti.
- `Setup.command`: installazione **opzionale** di una `.venv` indipendente, con
  Python 3.12 già installato e accesso a Internet. Utile se sposti la cartella.

Da Terminale, dentro questa cartella:

```sh
./Addestra.command --resume --steps 300000
./Addestra.command --output modelli_nuovi --steps 300000
./Avvia_RL.command --model modelli_nuovi/migliore.zip
./Valuta.command --model modelli_nuovi/migliore.zip --episodes 20
./Avvia_RL.command --headless
```

I launcher `.command` sono per macOS. Altrove crea un ambiente Python 3.12,
installa `requirements.txt` ed esegui direttamente gli script Python.

Nella finestra: **R** reset dal basso, **C** alterna policy e frenata, **A/D**
spinte esterne di prova, **Spazio** pausa, **Esc** chiude. Trascina con il mouse
per ruotare, usa la rotella per lo zoom. Quando la policy è disattivata viene
richiesta una frenata limitata: accelerazione zero, da sola, manterrebbe la
velocità comandata. Dopo la frenata il modello mantiene il riferimento di posizione.

## File e responsabilità

| File | Compito |
|---|---|
| `cart_pendolo.xml` | Geometria, masse, inerzie, gravità, attriti, giunti |
| `stepper.py` | Parametri STEP/DIR, integrazione, conteggio e attuatore equivalente |
| `ambiente.py` | Osservazioni, azioni, ricompensa, reset e fine episodio |
| `addestra.py` | SAC, selezione del modello, salvataggio e ripresa |
| `valuta.py` | Valutazione deterministica su semi di test |
| `simula.py`, `finestra.py` | Esecuzione e interfaccia grafica |
| `grafici.py` | Traiettoria CSV e confronto grafico posizione stimata/reale |
| `verifica.py` | Verifiche automatiche senza finestra |
| `TODO_STM32.md` | Interfaccia prevista e parti firmware ancora da implementare |

## Modello meccanico

La corsa fisica resta **±2,4 m** e il braccio principale arriva a **1 m** dal perno.
La vecchia guida visuale era stata accorciata senza modificare il giunto: qui è
nuovamente coerente con la corsa. I finecorsa logici intervengono a ±2,35 m; i
blocchi disegnati all'estremità rappresentano gli arresti meccanici, non il punto
elettrico esatto di intervento.

Carrello e supporti mantengono 1 kg complessivo. Il gruppo oscillante mantiene
0,2 kg: 0,152 kg sul braccio lungo, 0,038 kg sul corto, 0,010 kg su mozzo e testa.
Il **braccio corto di 0,25 m è un'ipotesi**, non una misura ricavata dalla foto.
MuJoCo ricalcola baricentro e inerzia dalle geometrie e dalle masse. Cambiare la
lunghezza da sola, mantenendo la massa esplicita, non cambia la massa: aggiornare
entrambe quando saranno disponibili le misure. Il perno è fuori dal volume del
carrello e permette la rotazione completa dell'asta.

Zero radianti significa **braccio lungo in alto**; π significa braccio lungo in
basso. Il braccio corto punta nel verso opposto. Questa convenzione deve essere
riprodotta dall'encoder angolare e non dipende dal verso dei fili del sensore.

La cinghia, le pulegge e il motore sono geometrie visuali statiche, senza denti
individuali o flessione della cinghia. La loro funzione dinamica è rappresentata
in `stepper.py`: non introduciamo migliaia di contatti inutili all'addestramento.
Il limite di forza include in modo aggregato il gruppo motore/trasmissione;
inerzia rotorica, curva coppia/velocità e risonanze non sono identificate.

## Da accelerazione a microstep

La policy restituisce un numero `a` in [-1, 1]. Ogni sottopasso fisico:

1. `accelerazione = a * accelerazione_max`;
2. si integra la velocità, limitandola a `velocita_max`;
3. si integra la posizione con la velocità media del sottopasso;
4. si quantizza il riferimento in microstep, conservando il resto frazionario;
5. si calcola una forza equivalente di inseguimento, limitata a `forza_max`.

La forza è `Kp*(x_riferimento-x_reale) + Kd*(v_comando-v_reale)`.
È un'**approssimazione della risposta meccanica**, non un regolatore da copiare
sulla STM32: la simulazione conosce x reale, mentre lo stepper reale può essere
privo di encoder. I guadagni devono essere identificati e validati sul banco.
La forza massima costante è una semplificazione, soprattutto ad alta velocità.

Parametri provvisori in `ParametriStepper`:

| Parametro | Valore iniziale |
|---|---:|
| Passo della cinghia | 2 mm |
| Denti puleggia | 20 |
| Passi interi/giro | 200 |
| Microstep | 16 |
| Velocità massima | 3 m/s |
| Accelerazione massima | 12 m/s² |
| Forza equivalente massima | 20 N |
| Rigidezza / smorzamento | 2000 N/m / 80 N s/m |
| Errore di inseguimento massimo | 5 cm |

Questi numeri non sono specifiche del tuo hardware. Con l'accoppiamento diretto:
`metri_impulso = passo_cinghia * denti / (passi_giro * microstep)`.
I valori iniziali danno 12,5 µm/impulso e, a 3 m/s, 240.000 impulsi/s. La prima
quantità è una **risoluzione nominale**, non un'accuratezza; la seconda impone di
verificare driver, temporizzazioni STEP/DIR e timer prima del trasferimento.

Il modello accumula gli impulsi a 500 Hz, senza emettere fronti elettrici reali.
Sul firmware un timer deve generare i fronti a frequenza molto più alta; non
bisogna emettere tutti gli impulsi insieme ogni 20 ms.

## Posizione disponibile alla rete

Il reset assume l'homing già completato e il conteggio inizializzato correttamente.
La rete usa **posizione da conteggio** e **velocità da differenza dei conteggi**,
non la posizione vera di MuJoCo. Angolo e velocità angolare sono ideali, senza
rumore o latenza in questa prima versione.

| Indice | Ingresso float32 | Scala |
|---:|---|---|
| 0 | Posizione stimata dai microstep | divisa per 2,4 m |
| 1 | sin(angolo) | nessuna |
| 2 | cos(angolo) | nessuna |
| 3 | Velocità stimata dai conteggi | divisa per 3 m/s |
| 4 | Velocità angolare | divisa per 10 rad/s |
| 5 | Velocità comandata interna | divisa per 3 m/s |

Il sesto ingresso rende visibile lo stato del generatore di traiettoria. Non rende
osservabili eventuali passi persi o tutta la dinamica interna del motore: resta
un'osservazione parziale della meccanica. Serve un nuovo addestramento.

La verità MuJoCo serve soltanto a calcolare ricompensa, successo e diagnostica,
e a simulare i finecorsa. Se l'errore riferimento/realtà supera 5 cm, l'episodio
termina con `inseguimento_fuori_modello`. È un criterio di esclusione delle
condizioni non rappresentate bene: **non è un rilevatore di passi persi disponibile
sul banco senza sensore**. Non simula perdita di sincronismo discreta dello stepper.

## RL e prestazioni

SAC resta l'algoritmo scelto. Addestramento stocastico con replay buffer e due
critici; esecuzione deterministica della sola media dell'attore, seguita da tanh.
La rete dell'attore è **6 → 128 ReLU → 128 ReLU → 1 tanh**.
Il ramo deterministico ha 17.537 parametri: 70.148 byte di pesi float32, esclusi
codice, buffer e runtime. È una stima della rete, non una misura di occupazione
STM32Cube.AI o del tempo di inferenza.

Fisica a 500 Hz, policy a 50 Hz, episodi da 20 s, sempre inizializzati dal basso
con piccole perturbazioni. Ricompensa per verticalità, posizione centrale e
velocità angolare ridotta, più un piccolo costo quadratico dell'accelerazione.
Arresto per finecorsa o eccessivo errore di inseguimento: penalità di 5.
Successo: almeno 3 s consecutivi con |angolo| < 12°, |x| < 1,5 m,
|velocità angolare| < 1 rad/s e |velocità carrello| < 0,75 m/s.

La rete piccola, PyTorch a un thread, un aggiornamento ogni quattro transizioni e
il modello equivalente senza contatti della cinghia contengono il costo CPU.
Nel ciclo fisico si usano saturazioni scalari; le osservazioni vengono costruite
solo alle decisioni. I controlli numerici rimangono attivi.

`modelli/migliore.zip` viene scelto per ricompensa media su cinque semi 1000–1004.
`ultimo.zip` e `esperienze.pkl` permettono la ripresa; `progresso.csv` registra
l'andamento. `valuta.py` usa invece semi 2000 e successivi. I risultati sono validi
solo nel modello nominale; semi diversi variano le condizioni iniziali, non le
masse o il motore. Non è ancora presente domain randomization.

Le firme SHA-256 di XML, ambiente e stepper vengono salvate in `config.json`.
Ripresa, valutazione e viewer rifiutano checkpoint con firme diverse: se modifichi
questi file, anche soltanto i commenti, addestra in una nuova cartella. Questo
controllo conservativo evita di presentare risultati ottenuti con un'altra dinamica.

## Prima del banco reale

Misurare corsa utile, entrambi i bracci rispetto al perno, masse/baricentro,
puleggia, cinghia, microstep e caratteristiche del motore/driver. Poi misurare
velocità/accelerazione sostenibili, risposta e attriti; aggiornare il modello e
riaddestrare. Implementare homing, encoder angolare, conteggio impulsi e arresti
indipendenti dalla policy. Introdurre rumori, ritardi e variabilità basati su
queste misure, quindi valutare anche su parametri diversi da quelli di training.

Gli esportatori C e ONNX precedenti restano intatti ma sono **specifici della
vecchia rete a cinque ingressi e dell'uscita forza**. Non usarli senza adattare
validazioni, normalizzazione e semantica dell'uscita. Nessun nuovo firmware o
ONNX viene prodotto da questa versione.

## Riferimenti

- [MuJoCo: attuazione e limitazione della forza](https://github.com/google-deepmind/mujoco/blob/main/doc/modeling.rst)
- [Stable-Baselines3: SAC](https://stable-baselines3.readthedocs.io/en/master/modules/sac.html)
- [Homing tramite finecorsa](https://www.pololu.com/docs/0J71/4.14)

## Prima esecuzione inclusa (11 settembre 2026)

Addestramento di 300.000 decisioni, seed 42, circa 5,2 minuti su questo computer.
Il checkpoint selezionato è quello a 200,000 decisioni.
Test separato su 20 semi: **100% di successi**, ricompensa media
933.21; nessuna terminazione anticipata in queste prove.
Partenza esattamente dal basso: 18,36 s di equilibrio continuo su 20 s.
Questi risultati non includono variabilità dei parametri fisici o prove hardware.

Dettagli in `risultati/valutazione.json`, verifica meccanica in
`risultati/verifica.json`, traiettoria in `risultati/traiettoria.csv` e
grafico in `risultati/controllo_stepper.png`. La grafica interattiva non è stata
verificata visivamente in questa sessione; sono stati verificati il launcher in
modalità headless e la compilazione del codice della finestra.

## Visualizzazione continua

`Avvia_RL.command` esegue ora il controllo senza limite di durata. La finestra
imposta `max_steps = infinito` soltanto sulla propria istanza: non cambia la
fisica o la policy e non richiede riaddestramento. Il tempo simulato viene
sincronizzato al tempo del PC, con decisioni ogni 20 ms; in caso di sovraccarico
la simulazione può rallentare (non è un sistema hard real-time).

R resetta manualmente, Spazio mette in pausa. Dopo finecorsa o errore di
inseguimento la simulazione si arresta e richiede R; non riparte automaticamente.
Le impostazioni personali di font e disturbo sono conservate.

Addestramento, valutazione e `--headless` mantengono episodi finiti secondo
`ambiente.py`, attualmente modificato dall'utente a 10.000 decisioni (200 s).
I risultati del primo addestramento riportati sopra si riferiscono ancora a
1.000 decisioni (20 s). La firma dei checkpoint è stata migrata verificando che
questa fosse l'unica modifica all'ambiente; la migrazione è registrata in config.json.

## Spostamento manuale di 10 cm

Freccia sinistra/destra richiede -/+10 cm rispetto alla posizione corrente da
conteggio. Una pressione avvia una sola manovra; auto-repeat e altre richieste
durante il movimento vengono ignorati. In pausa la richiesta attende la ripresa.
Il riferimento viene raggiunto gradualmente (massimo 0,3 m/s e 2 m/s²); non viene
modificata direttamente la posizione fisica. La tolleranza sul conteggio è 0,5 mm.

Durante la manovra la policy è temporaneamente sospesa: l'equilibrio del pendolo
non è garantito. Al termine ritorna il controllo precedente: se SAC era attivo,
può spostare nuovamente il carrello per bilanciare. Usa C prima della manovra se
vuoi mantenerlo nella nuova posizione senza SAC. C o R annullano la manovra.
Le richieste troppo vicine ai finecorsa vengono rifiutate; gli arresti fisici
restano attivi. Dopo 10 s senza arrivo il comando manuale termina con frenata e
policy disattivata. Non occorre riaddestrare né cambiare le firme dei checkpoint.
