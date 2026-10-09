# Stabilizzazione dall’alto con disturbi — 1 ottobre 2026

Il controllo LQR (`equilibrio.py` e file collegati) è stato rimosso: resta solo SAC.
Tutti gli episodi partono dall’alto; lo swing-up non è ancora incluso.

**Perché con i disturbi la rete non stabilizzava più.** Nel vecchio
`disturbi.toml` il rumore era 0,05 × intensità 0,25 = 12,5 mm e 0,0125 rad
(0,72°). Il problema principale però non era il valore, ma come venivano
calcolate le velocità: differenza di due letture rumorose ogni 20 ms.
Con 7,5 mm di rumore la velocità del carrello ha circa 0,53 m/s di rumore
(il massimo comandabile è 0,48 m/s); con 0,75° la velocità angolare ha circa
0,9 rad/s di rumore. La rete riceveva quasi solo rumore.

**Correzione.**

1. `disturbi.toml`: rumore di posizione 5–10 mm e dell’encoder 0,5–1°
   (deviazione standard di ogni lettura a 50 Hz, estratta a ogni episodio
   nell’intervallo). Non è più scalato da `intensita`.
2. `stima.py` + `stimatore.toml`: stimatore dello stato. Prevede il moto con la
   fisica nota (velocità applicata dalla rampa per il carrello, equazione del
   pendolo per l’angolo) e lo corregge poco con le misure. La rete riceve gli
   stessi sei ingressi di prima, ma stimati invece che derivati. Lo stesso
   algoritmo (poche righe) va portato sul firmware: vedi `TODO_STM32.md`.
3. Nuova rete addestrata con i disturbi: `modelli_disturbi/migliore.zip`.

```sh
./Avvia_RL.command --model modelli_disturbi/migliore.zip
./Avvia_RL.command --model modelli_disturbi/migliore.zip --nominale
./Valuta.command --model modelli_disturbi/migliore.zip --episodes 20 --output risultati/valutazione_disturbi.json
```

I risultati delle prove sono in `TEST_ESEGUITI.md`.

---

# Pendolo — SAC con velocità in Hz

Questa cartella deriva da `cart_pendolo_stepper_rl` fornita dall'utente.
Conserva geometria XML, algoritmo SAC, rete con due strati da 128 neuroni,
comandi della finestra, launcher macOS, valutazione e grafici. Sostituisce il
comando di accelerazione e il modello di inseguimento STEP con una conversione
di velocità seguita da una rampa di frequenza. Il progetto originale resta intatto.

## Avvio

I launcher cercano prima `.venv`, poi l'ambiente del progetto originale adiacente
`../cart_pendolo_stepper_rl/.venv`, poi `../cart_pendolo_rl/.venv`. Non modificano
questi ambienti. Per un ambiente indipendente eseguire `Setup.command`.

- `Verifica.command`: test e verifica geometrica senza finestra.
- `Avvia_Manuale.command`: apre la simulazione senza una rete addestrata.
- `Addestra.command`: addestra SAC per 300.000 decisioni.
- `Avvia_RL.command`: esegue la migliore rete addestrata.
- `Valuta.command`: valuta su 20 semi separati da quelli di selezione.
- `Grafici.command`: produce CSV e grafici della traiettoria.

Da Terminale, all'interno di questa cartella:

```sh
./Addestra.command --steps 300000
./Addestra.command --resume --steps 300000
./Addestra.command --output modelli_altri --steps 300000
./Avvia_RL.command --model modelli_altri/migliore.zip
./Avvia_RL.command --manual
./Avvia_RL.command --headless --model modelli/ultimo.zip
./Valuta.command --model modelli/ultimo.zip --episodes 20
./Grafici.command --model modelli/ultimo.zip
```

`migliore.zip` viene salvato alla valutazione periodica (ogni 10.000 decisioni).
`ultimo.zip` e il replay buffer vengono salvati anche alla fine o con Ctrl+C.
Per un addestramento più corto di 10.000 decisioni usare `ultimo.zip` oppure
ridurre `--eval-every`. I vecchi checkpoint e le vecchie statistiche non vengono
copiati: hanno un contratto diverso e non misurano le prestazioni di questa versione.

## File

| File | Responsabilità |
|---|---|
| `stepper.toml` | Parametri hardware e conversione |
| `stepper.py` | Conversione dell'azione e rampa della frequenza STEP |
| `cart_pendolo.xml` | Geometria originale, con attuatore di velocità |
| `ambiente.py` | Osservazioni, azione, ricompensa, finecorsa e reset |
| `addestra.py` | SAC, checkpoint, ripresa, selezione della rete |
| `simula.py`, `finestra.py` | Simulazione e interfaccia grafica |
| `valuta.py` | Valutazione su semi 2000 e successivi |
| `grafici.py` | CSV e grafici di Hz, posizione e velocità |
| `verifica.py`, `test_versione.py` | Verifiche automatiche |
| `TODO_STM32.md` | Contratto previsto per il firmware |
| `modelli/`, `risultati/` | Nuovi checkpoint e risultati |

## Uscita della rete e configurazione

La rete emette **un solo numero u in [-1, 1]**: il segno sceglie il verso,
il modulo la velocità. Non serve una seconda uscita di direzione.
Con minimo e zona morta nulli: `frequenza_hz = u × frequenza_max_hz`.
Hz significa **impulsi STEP al secondo**, non giri al secondo dell'albero.
Il segno è una convenzione software; sul driver si usano il modulo per STEP
ed il segno per DIR. `u=0` richiede una frenata a rampa fino all'arresto.

La configurazione attuale contiene valori da confermare sul banco:

| Parametro | Valore |
|---|---:|
| Passi interi per giro | 200 |
| Angolo passo | 1,8° |
| Microstepping | 32 |
| Passo cinghia | 2 mm |
| Denti puleggia | 48 |
| Rapporto giri motore / giri puleggia | 1 |
| Frequenza minima richiesta in movimento | 3.200 Hz |
| Frequenza massima | 32.000 Hz |
| Rampa di salita e discesa | 1.600.000 Hz/s |

Con questi valori ci sono 6.400 impulsi/giro e 0,015 mm/impulso: 32.000 Hz
corrispondono a 0,48 m/s. Per esempio `u=-0,5` richiede -17.600 Hz e -0,264 m/s.
La rampa configurata equivale a 24 m/s² del riferimento di velocità;
non è un limite misurato del motore.
Cambiare i valori in `stepper.toml` prima dell'addestramento.

```
impulsi_per_giro = passi_per_giro × microstepping
metri_per_impulso = passo_cinghia_m × denti_puleggia
                   / (impulsi_per_giro × rapporto_trasmissione)
v_carrello = frequenza_hz × metri_per_impulso
```

Passi/giro e angolo vengono verificati: il loro prodotto deve essere 360°.
Microstepping e trasmissione devono corrispondere all'hardware.
Con zona morta d, per `|u| <= d` si restituisce zero; altrimenti:
`f = segno(u) × [f_min + (|u|-d)/(1-d) × (f_max-f_min)]`.
Un minimo positivo introduce un salto nella richiesta, smussato dalla rampa.
Durante partenza, frenata e inversione il comando attraversa anche frequenze
inferiori al minimo. Valori fuori [-1,1] sono saturati; NaN e infinito sono rifiutati.

`rampa_hz_s` deve essere finito e positivo. A ogni passo fisico (2 ms):

```text
f_applicata += clip(f_richiesta - f_applicata, -rampa_hz_s * dt, +rampa_hz_s * dt)
```

La rampa vale nei due versi e non supera la frequenza richiesta. Con i valori
attuali 0 -> 32.000 Hz richiede 0,02 s; +32.000 -> -32.000 Hz richiede 0,04 s.
Il commento storico nel TOML indica ancora 0,5 s; fa fede il valore numerico.
Reset, finecorsa e fine episodio azzerano subito il comando e lo stato della rampa.

## Simulazione e osservazioni

Il percorso è `rete → Hz richiesti → rampa → Hz applicati → m/s → attuatore MuJoCo`.
L'attuatore di velocità usa un guadagno numerico fisso `kv=200` nell'XML:
forza equivalente = guadagno × errore di velocità. Non richiede di identificare
coppia, rigidezza o inerzia del rotore. La rampa limita il riferimento, non
l'accelerazione fisica effettiva del carrello. Non simula
impulsi individuali, perdita di passi o limiti di coppia. Ha un breve
transitorio di inseguimento e non rappresenta una velocità imposta esattamente.
La dinamica del pendolo è ancora accoppiata fisicamente al carrello.

La geometria è quella della cartella originale fornita, senza sostituirla con
un'altra revisione. Masse e dimensioni non confermate restano ipotesi.
Finecorsa e limite di corsa vengono letti dall'XML. Fisica a 500 Hz, controllo
a 50 Hz, episodi di 20 s; il viewer usa durata continua fino a reset/arresto.

Le osservazioni sono sei: cinque da sensori ideali e una dallo stato della rampa:

1. posizione carrello / limite di corsa;
2. sin(angolo);
3. cos(angolo);
4. velocità carrello / velocità massima configurata;
5. velocità angolare / 10 rad/s.
6. frequenza applicata dopo la rampa / frequenza massima.

Zero angolare è il pendolo in alto, π il pendolo in basso. La frequenza della
rampa è uno stato interno noto al controllore. Sul banco le grandezze fisiche
devono essere misurate o stimate con
normalizzazioni identiche: il conteggio STEP da solo non misura lo slittamento.
La rete è **6 → 128 ReLU → 128 ReLU → 1 tanh** e richiede nuovo addestramento.
Le firme di XML, ambiente, stepper e TOML impediscono il caricamento di
checkpoint incompatibili dopo modifiche al modello o ai parametri.
I checkpoint precedenti senza rampa non possono essere ripresi. Avviare, ad esempio,
`./Addestra.command --output modelli_rampa --steps 300000` per conservare quelli esistenti.

## Addestramento con vibrazioni e disturbi

Il profilo opzionale `disturbi.toml` aggiunge disturbi al modello nominale gia
allineato al banco. Non cambia l'XML, i parametri hardware, i sei ingressi
della rete o il firmware. Senza profilo, il percorso nominale e i checkpoint
esistenti rimangono compatibili.

Per addestrare una nuova policy robusta, dalla cartella di questo progetto:

```sh
./Addestra.command --disturbi disturbi.toml --output modelli_disturbi --steps 450000
./Addestra.command --resume --output modelli_disturbi --steps 300000
./Valuta.command --model modelli_disturbi/migliore.zip --episodes 20 --output risultati/valutazione_disturbi.json
./Avvia_RL.command --model modelli_disturbi/migliore.zip
./Avvia_RL.command --model modelli_disturbi/migliore.zip --nominale
```

La ripresa usa il profilo salvato nel checkpoint: modificare il TOML non cambia
silenziosamente il training in corso. Per un profilo differente avviare un nuovo
addestramento in una cartella diversa. La selezione confronta il nominale e il
disturbato sui medesimi cinque stati iniziali, e massimizza la minore delle due
durate stabili peggiori. La valutazione finale usa semi separati (2000 e successivi),
riporta entrambe le suite e le durate di equilibrio. Il successo richiede
l’intero episodio stabile; il criterio storico dei tre secondi resta solo nel
campo diagnostico `three_second_success`.

Per misurare subito quanto e sensibile la rete gia addestrata:

```sh
./Valuta.command --model modelli/migliore.zip --disturbi disturbi.toml --episodes 20 --output risultati/stress_rete_esistente.json
./Avvia_RL.command --model modelli/migliore.zip --disturbi disturbi.toml
./Grafici.command --model modelli/migliore.zip --disturbi disturbi.toml --output risultati/grafici_disturbi
```

Valutazione, viewer e grafici usano automaticamente il profilo salvato con una
nuova policy robusta. `--disturbi altro.toml` permette uno stress diverso;
`--nominale` lo disabilita. Viewer e grafici accettano anche `--seed`.

### Cosa viene simulato

- **Tre modi oscillanti smorzati**, eccitati dall'accelerazione del riferimento
  del motore e da rumore correlato nel tempo. Cinghia e motore applicano forze
  al carrello; l'asta applica una coppia al pendolo. Ogni modo segue
  `y'' + 2*zeta*w*y' + w²*y = w²*eccitazione`. La risposta viene convertita in
  forza/coppia tramite `ampiezza*tanh(y)`, per limitarne il valore.
- **Eccitazione casuale correlata**, con tempo caratteristico `correlazione_s`:
  evita di rappresentare tutte le vibrazioni come campioni bianchi indipendenti.
- **Variazioni per episodio** di masse e inerzie (scalate insieme), smorzamento
  dei due giunti, guadagno del servo, frequenze dei modi, smorzamento dei modi,
  ampiezze, offset angolare e ritardo del comando. Nessuna deriva cumulativa
  dei parametri tra reset; i coefficienti derivati MuJoCo vengono ricalcolati.
- **Misure perturbate** di posizione e angolo; l'angolo e quantizzato ai
  conteggi encoder configurati. Le misure passano dallo stimatore `stima.py`:
  le velocita NON sono derivate dalle misure. Seno e coseno usano lo stesso
  angolo stimato; le scale di normalizzazione sono sempre quelle nominali. Lo stato fisico, la ricompensa
  e i finecorsa non usano le misure rumorose. Il modello non aggiunge passi persi.

I modi sono integrati a 500 Hz con soluzione esatta per eccitazione costante
nel singolo passo fisico; il comando della rete resta a 50 Hz. Si richiedono
almeno dieci passi fisici per periodo: con 2 ms il limite configurabile e 50 Hz.
Non aumentare solo la frequenza dei modi oltre tale soglia. I modi sono
**disturbi equivalenti**, non un modello strutturale flessibile dell'asta,
della cinghia o delle bobine del motore. Non riproducono automaticamente
frequenze, accoppiamenti ed energia del sistema reale.

### Come regolare il profilo

I valori iniziali sono ipotesi di stress, non misure: cinghia 8–18 Hz,
motore 20–40 Hz, asta 3–10 Hz; `zeta` tra 0,08 e 0,25. A intensita 1 i
limiti massimi sono rispettivamente 0,20 N, 0,10 N e 0,003 Nm; a ogni episodio
l'ampiezza di ciascun modo e estratta tra il 50% e il 100% del suo limite.

Il profilo fornito usa `intensita = 0.25`. Questo fattore scala ampiezze,
variazioni dei parametri, offset angolare e ritardo massimo; non scala le
frequenze, `zeta`, il tempo di correlazione, la risoluzione dell'encoder
e il rumore dei sensori (`rumore_posizione_m`, `rumore_angolo_gradi`).
Con questi valori il ritardo e 0 o 2 ms, arrotondato per difetto al passo fisico.
`intensita = 0` restituisce esattamente l'ambiente nominale, anche nei sensori.

Per cominciare cambia solo `intensita`, confrontando ad esempio 0,2, 0,35 e 0,6
nelle valutazioni. Usa frequenze misurate quando disponibili e amplia i range
solo quanto giustificato dalle osservazioni. Vibrazioni ad alta frequenza possono
essere aliasate nei campioni della policy a 50 Hz: una misura a 50 Hz non basta
per identificare modi a 40 Hz.

Ogni rapporto disturbato registra parametri estratti, forza e coppia finali.
I seed rendono le prove ripetibili; chiamare piu volte `observation()` non genera
nuovo rumore. Le spinte manuali si sommano ai disturbi; reset e fine episodio
svuotano la coda dei comandi e le forze applicate.

Test automatici: `python -m unittest test_versione test_disturbi` nell'ambiente
Python del progetto. Verificano determinismo, confronto nominale, oscillazioni
smorzate, ritardo, rumore separato dalla fisica, limiti e contratti checkpoint.
La gestione dei parametri segue la [documentazione MuJoCo](https://mujoco.readthedocs.io/en/latest/programming/simulation.html).

`env.step()` mantiene l'interfaccia Gymnasium. Nel dizionario `info`:

- `motor_frequency_hz`: comando firmato dopo la rampa, azzerato a fine episodio;
- `requested_frequency_hz`: richiesta prima della rampa e degli arresti;
- `step_frequency_hz`, `direction`: modulo e verso separati;
- `command_velocity_m_s`, `cart_velocity_m_s`: riferimento e velocità simulata.

Il convertitore è utilizzabile anche senza MuJoCo:

```python
from stepper import ControlloStepper
motore = ControlloStepper()
f_hz = motore.azione_a_hz(-0.5)
comando = motore.avanza(-0.5, dt=0.002)  # chiamare a ogni passo della rampa
# comando.frequenza_step_hz, comando.direzione
motore.reset()  # azzeramento immediato
```

## Finestra

R reset, C alterna SAC/arresto, A/D applicano spinte, Spazio pausa, Esc chiude.
Mouse per ruotare, rotella per zoom. Frecce: movimento manuale di +/-10 cm;
durante il movimento la policy viene sospesa e poi ripristinata. Il controllo
manuale usa un riferimento di velocità proporzionale all'errore, limitato a
0,3 m/s e al massimo configurato, seguito dalla stessa rampa usata dalla policy.
Con un minimo Hz elevato la precisione manuale può peggiorare: dopo 10 s
scatta il timeout con richiesta di arresto. I finecorsa fermano la simulazione
finché non si preme R. Il display mostra il comando STEP firmato in Hz.
Senza rete (`--manual`) C non può attivare SAC: restano disponibili i comandi manuali.

## Verifica e limiti

`Verifica.command` controlla conversione, configurazione, Gymnasium, risposta
meccanica, arresto, finecorsa, episodio completo, 1.805 configurazioni geometriche
interne alla corsa e manovre manuali nei due versi. Il rapporto è scritto in
`risultati/verifica.json`. `TEST_ESEGUITI.md` riporta le prove effettivamente
eseguite per questa consegna. Un breve test di addestramento verifica la pipeline,
non dimostra che la rete abbia imparato lo swing-up. Non sono inclusi risultati
di successo della vecchia policy né una nuova rete dichiarata pronta per il banco.
