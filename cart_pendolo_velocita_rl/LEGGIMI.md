# Pendolo — SAC con velocità in Hz

Questa cartella deriva da `cart_pendolo_stepper_rl` fornita dall'utente.
Conserva geometria XML, algoritmo SAC, rete con due strati da 128 neuroni,
comandi della finestra, launcher macOS, valutazione e grafici. Sostituisce il
comando di accelerazione e il modello di inseguimento STEP con una conversione
semplice di velocità. Il progetto originale resta intatto.

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
| `stepper.py` | Un'azione firmata diventa una frequenza STEP |
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
ed il segno per DIR. `u=0` significa arresto degli impulsi.

La configurazione iniziale riprende i valori del `stepper.py` originale,
che erano esplicitamente provvisori:

| Parametro | Valore |
|---|---:|
| Passi interi per giro | 200 |
| Angolo passo | 1,8° |
| Microstepping | 8 |
| Passo cinghia | 2 mm |
| Denti puleggia | 48 |
| Rapporto giri motore / giri puleggia | 1 |
| Frequenza minima | 0 Hz |
| Frequenza massima | 50.000 Hz |

Con questi valori ci sono 1.600 impulsi/giro e 0,06 mm/impulso: 50.000 Hz
corrispondono ai **3 m/s massimi del vecchio codice**, non a una velocità
verificata sul banco. Per esempio `u=-0,5` produce -25.000 Hz e -1,5 m/s.
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
Un minimo positivo introduce un salto fra arresto e movimento. Non è una
velocità minima negativa. Valori fuori [-1,1] sono saturati; NaN e infinito
sono rifiutati.

## Simulazione e osservazioni

Il percorso è `rete → Hz firmati → m/s carrello → attuatore MuJoCo`.
L'attuatore di velocità usa un guadagno numerico fisso `kv=200` nell'XML:
forza equivalente = guadagno × errore di velocità. Non richiede di identificare
coppia, rigidezza, accelerazione massima o inerzia del rotore. Non simula
impulsi individuali, perdita di passi o limiti di coppia. Ha un breve
transitorio di inseguimento e non rappresenta una velocità imposta esattamente.
La dinamica del pendolo è ancora accoppiata fisicamente al carrello.

La geometria è quella della cartella originale fornita, senza sostituirla con
un'altra revisione. Masse e dimensioni non confermate restano ipotesi.
Finecorsa e limite di corsa vengono letti dall'XML. Fisica a 500 Hz, controllo
a 50 Hz, episodi di 20 s; il viewer usa durata continua fino a reset/arresto.

Le osservazioni sono cinque, da sensori ideali dello stato simulato:

1. posizione carrello / limite di corsa;
2. sin(angolo);
3. cos(angolo);
4. velocità carrello / velocità massima configurata;
5. velocità angolare / 10 rad/s.

Zero angolare è il pendolo in alto, π il pendolo in basso. Non c'è più lo
stato interno del generatore di accelerazione né la posizione da conteggio
simulato. Sul banco queste grandezze devono essere misurate o stimate con
normalizzazioni identiche: il conteggio STEP da solo non misura lo slittamento.
La rete è **5 → 128 ReLU → 128 ReLU → 1 tanh** e richiede nuovo addestramento.
Le firme di XML, ambiente, stepper e TOML impediscono il caricamento di
checkpoint incompatibili dopo modifiche al modello o ai parametri.

`env.step()` mantiene l'interfaccia Gymnasium. Nel dizionario `info`:

- `motor_frequency_hz`: comando firmato, azzerato a fine episodio;
- `requested_frequency_hz`: richiesta prima degli arresti;
- `step_frequency_hz`, `direction`: modulo e verso separati;
- `command_velocity_m_s`, `cart_velocity_m_s`: riferimento e velocità simulata.

Il convertitore è utilizzabile anche senza MuJoCo:

```python
from stepper import ControlloStepper
motore = ControlloStepper()
f_hz = motore.azione_a_hz(-0.5)
comando = motore.comando(-0.5)
# comando.frequenza_step_hz, comando.direzione
```

## Finestra

R reset, C alterna SAC/arresto, A/D applicano spinte, Spazio pausa, Esc chiude.
Mouse per ruotare, rotella per zoom. Frecce: movimento manuale di +/-10 cm;
durante il movimento la policy viene sospesa e poi ripristinata. Il controllo
manuale usa un riferimento di velocità proporzionale all'errore, limitato a
0,3 m/s e al massimo configurato. Non è una traiettoria con accelerazione limitata.
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
