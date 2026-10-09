# Prova hardware-in-the-loop (HIL)

La simulazione MuJoCo gira sul Mac; **stimatore (`stima.c`) e rete X-CUBE-AI
girano sulla STM32**. Ogni 20 ms il Mac manda alla scheda posizione e angolo
simulati (con rumore e disturbi di `disturbi.toml`) e le velocità della rampa;
la scheda risponde con il comando, che pilota la simulazione. In parallelo il
Mac calcola `stima.py` e la rete PyTorch e confronta i risultati passo per passo.

Verifica: runtime ST e pesi caricati, stimatore in C su Cortex-M4, tempo di
inferenza reale, protocollo. **Non** verifica motore, driver, encoder e meccanica.

In modalità HIL il firmware non fa homing, non legge l'encoder e non comanda mai
il motore. Per sicurezza tieni comunque **spenta l'alimentazione del driver**.

## 1. Prendere il ramo

Dalla cartella `cart-inverted-pendulum` nel Terminale:

```sh
git stash -u                    # mette da parte le modifiche non salvate di main
git checkout stimatore-stm32
git push -u origin stimatore-stm32   # pubblica il ramo su GitHub
```

(Le modifiche messe da parte sono già tutte nel ramo; si recuperano con
`git checkout main && git stash pop` se servisse.)

## 2. Firmware in modalità HIL

1. Apri `Prova_controllo` in STM32CubeIDE.
2. In `Core/Src/main.c` cerca `#define HIL_MODE 0` e scrivi **`1`**.
3. *Project → Build Project* (martello). Devono comparire 0 errori.
4. Collega la Nucleo via USB e caricala: *Run → Run As → STM32 C/C++ Application*
   (oppure il pulsante verde Run).
5. Il LED verde LD2 resta spento finché il Mac non manda dati, poi lampeggia
   a ogni pacchetto.

Non serve rigenerare il codice da CubeMX. Se lo rigeneri, tutto il codice
HIL e dello stimatore è dentro i blocchi `USER CODE` e in `stima.c`, quindi resta.

## 3. Avviare la prova dal Mac

Chiudi ogni monitor seriale aperto sulla scheda (la porta sarebbe occupata).
Dalla cartella `cart_pendolo_velocita_rl`:

```sh
./HIL.command --ping                       # 1) verifica la connessione
./HIL.command                              # 2) 5 episodi da 20 s
./HIL.command --episodi 10 --secondi 60    # prova più lunga
./HIL.command --intensita 1                # disturbi al massimo
```

Si può anche fare doppio clic su `HIL.command` nel Finder (5 episodi da 20 s).
La prima volta installa `pyserial` nell'ambiente Python del progetto.
La porta viene trovata da sola; se non la trova: `ls /dev/cu.usbmodem*` e poi
`./HIL.command --porta /dev/cu.usbmodemXXXX`.

Con la seriale a 115200 baud ogni passo richiede circa 12–15 ms: un episodio
da 20 s dura circa 20–30 s reali.

Durante la prova, per ogni episodio viene stampato:

```
Episodio 1/5 seed 7000: OK | stabile 20.0 s | angolo max 2.5 deg | diff obs 1.0e-07 | diff azione 2.4e-07 | rete 450 us (max 470)
```

- **OK / CADUTO**: il pendolo, comandato dalla scheda, resta in equilibrio?
- **diff obs**: ingressi della rete calcolati dalla scheda contro `stima.py`
  (atteso < 1e-4: differenze di arrotondamento float32).
- **diff azione**: uscita X-CUBE-AI contro PyTorch sugli stessi ingressi
  (atteso < 1e-4; valori grandi = pesi non aggiornati o rete sbagliata).
- **rete**: tempo di `ai_network_run` misurato sulla scheda (deve stare
  molto sotto i 20 ms del periodo di controllo).

I risultati finiscono in `cart_pendolo_velocita_rl/risultati/hil/`:
`hil_<data>.csv` (ogni passo) e `hil_<data>.json` (riepilogo). Dimmi quando ci
sono e li analizzo io.

## 4. Tornare al funzionamento normale

Rimetti `#define HIL_MODE 0` in `main.c`, ricompila e ricarica la scheda
**prima** di riaccendere il driver del motore.

## Problemi comuni

| Messaggio | Causa probabile |
|---|---|
| `Nessuna scheda trovata` | Cavo solo di alimentazione, o porta diversa: usa `--porta` |
| `Resource busy` / porta occupata | Un monitor seriale (CubeIDE, screen, CoolTerm) è aperto |
| `La scheda non risponde` | Firmware con `HIL_MODE 0`, oppure scheda in pausa nel debugger: premi Resume o il tasto nero RESET |
| `Errore della scheda 2` | Rete X-CUBE-AI non inizializzata o inferenza fallita |
| `diff azione` grande | Pesi in `network_data_params.c` diversi dalla rete `--model` |

## Protocollo (per riferimento)

USART2 (ST-LINK VCP), 115200 8N1, little-endian. Ogni pacchetto:
`A5 5A | corpo | checksum`, con checksum = complemento a due della somma dei
byte del corpo.

- PC → scheda, 58 byte: `comando u8` (0 inizio episodio, 1 passo, 2 ping),
  `n u8`, `seq u32`, `v[10] f32` (m/s, una per ogni 2 ms), `x f32` (m),
  `theta f32` (rad), `applied_hz f32`.
- Scheda → PC, 60 byte: `seq u32`, `errore u32`, `azione f32`,
  `richiesta_hz f32`, `osservazione[6] f32`, `stima x, theta, omega f32`,
  `rete_us u32`, `totale_us u32`.

Una richiesta con lo stesso `seq` della precedente non viene rieseguita: la
scheda rimanda la risposta precedente (ritrasmissione dopo un timeout).
Test del protocollo senza scheda: `python3 Prova_controllo/tests/test_hil.py`.
