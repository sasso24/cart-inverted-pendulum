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
| Rampa di salita e discesa | 64.000 Hz/s |

Con questi valori ci sono 6.400 impulsi/giro e 0,015 mm/impulso: 32.000 Hz
corrispondono a 0,48 m/s. Per esempio `u=-0,5` richiede -17.600 Hz e -0,264 m/s.
La rampa iniziale equivale a 0,96 m/s² del riferimento di velocità;
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
attuali 0 -> 32.000 Hz richiede 0,5 s; +32.000 -> -32.000 Hz richiede 1 s.
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
