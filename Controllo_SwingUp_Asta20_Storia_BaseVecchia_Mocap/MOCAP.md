# Angolo dal sistema di tracciamento

```
tracciamento (marker sull'asta) ──► driver ROS 2 (vrpn_mocap, mocap4r2, …)
    ──► geometry_msgs/PoseStamped del corpo rigido dell'asta
    ──► nodo ponte_mocap (ros/pendolo_mocap) ──USB──► ST-LINK ──► USART2 della Nucleo
                                             ◄────── telemetria a 50 Hz ◄──────
```

Il firmware usa a ogni periodo della rete (20 ms) l'ultimo angolo arrivato. Rete, pesi,
stimatore, homing e rampa sono quelli di `Controllo_SwingUp_Asta20_Storia_BaseVecchia`.

## Convenzione dell'angolo

La stessa dell'addestramento (`cart_pendolo_asta20.xml`) e dell'encoder con `ENCODER_SIGN -1`:

- **0** = braccio lungo (0,78 m) in alto, **±180°** = appeso;
- **positivo** quando la punta del braccio lungo va verso **+x**, cioè verso il
  **finecorsa destro guardando il banco da dietro** (il verso in cui crescono i passi del carrello).

`ponte_mocap` la ricava dal solo **orientamento** del corpo rigido montato sull'asta (la
posizione del carrello non serve). Va detto al nodo, nel sistema del tracciamento:

| parametro | cosa | tipico |
|---|---|---|
| `asse_su` | verticale verso l'alto | `[0.0, 0.0, 1.0]` se il driver converte in ROS (z in alto); `[0.0, 1.0, 0.0]` con Motive in Y-up |
| `asse_carrello` | binario, verso il finecorsa destro visto da dietro | dipende da come è orientato il banco rispetto al tracciamento |
| `direzione_corpo` | direzione dell'asta nel corpo rigido | `[0,0,0]` = calibrazione all'avvio con il pendolo **fermo appeso** (1 s); il nodo stampa il valore da mettere qui |

Non serve sapere come è definito il corpo rigido nel software del tracciamento, né montarlo
dritto: la calibrazione da appeso lo assorbe.

**Verifica del verso (una volta, a motore spento):** con il nodo avviato e il pendolo appeso,
`ros2 topic echo /pendolo/angolo` dà circa ±3,14. Spostare a mano la punta del braccio lungo
verso il finecorsa destro (da dietro): l'angolo deve **scendere da +3,14** verso +1,57. Se
invece va da -3,14 verso -1,57, invertire `asse_carrello`. Se il nodo avvisa "asta fuori dal
piano di oscillazione", `asse_su` o `asse_carrello` sono sbagliati. Con l'encoder ancora
montato il confronto è diretto: `theta_deg` contro `encoder_theta_deg` nelle Live Expressions
(o `theta` contro `theta_encoder` su `/pendolo/stm32`).

## Installare il pacchetto ROS 2

```sh
cd ~/ros2_ws/src
ln -s "<repo>/Controllo_SwingUp_Asta20_Storia_BaseVecchia_Mocap/ros/pendolo_mocap" .
sudo apt install python3-serial          # oppure pip install pyserial
cd ~/ros2_ws && colcon build --packages-select pendolo_mocap && source install/setup.bash
sudo usermod -aG dialout $USER           # accesso a /dev/ttyACM*, poi rifare il login
```

Solo `rclpy` e messaggi standard (`geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`):
nessun messaggio personalizzato. Provato senza ROS (`tests/test_collegamento.py`), non ancora
su una installazione ROS 2 vera.

## Sul banco

1. Driver del tracciamento avviato, corpo rigido dell'asta visibile (`ros2 topic hz <topic>` ≈ 200 Hz).
2. Copiare e correggere `ros/pendolo_mocap/config/ponte_mocap.yaml` (porta, `topic_posa`, assi), poi
   ```sh
   ros2 launch pendolo_mocap ponte_mocap.launch.py parametri:=/percorso/ponte_mocap.yaml
   ```
   Pendolo **fermo appeso** finché il nodo scrive `calibrato`.
3. Accendere/resettare la Nucleo. LD2 lampeggia finché non arrivano angoli; poi la scheda
   vuole il pendolo fermo appeso per 1 s (riferimento: offset in `theta_offset_rad`, di solito
   pochi decimi di grado), poi fa l'homing. Il nodo scrive nel log ogni cambio di stato e di
   `controllo_errore`.
4. B1 come prima (LEGGIMI.md, "Uso sul banco"). Se gli angoli mancano da più di 50 ms il
   controllo si ferma con `controllo_errore = 12`.

Topic pubblicati: `/pendolo/angolo` (std_msgs/Float64, rad, quello mandato) e `/pendolo/stm32`
(sensor_msgs/JointState, un messaggio per periodo, nomi `theta`, `theta_encoder`, `stima_theta`,
`stima_x`, `azione`, `eta_ms`, `ritardo_ms`; comodo in PlotJuggler). Servizio `~/calibra`
(std_srvs/Trigger) per ripetere la calibrazione del nodo: intanto non partono angoli.

Chiudere ogni monitor seriale sulla porta dell'ST-LINK: la porta la usa il nodo.

## Cosa fa il firmware con l'angolo

- Ricezione a interrupt (`Mocap_UartIrq`, `USART2_IRQHandler` scritto a mano in
  `stm32f4xx_it.c`: **non rigenerare da CubeMX**, come già per X-CUBE-AI).
- Pacchetti con checksum, tipo o valore errati: scartati interi, risincronizzazione su `A5 5A`.
  Corpo rigido perso (stato 0 dal nodo): il campione precedente invecchia.
- **Timeout 50 ms** (`MOCAP_TIMEOUT_MS`, 10 campioni persi): angolo non valido; in controllo
  arresto con errore 12, da fermo B1 rifiutato con errore 12 e stimatore ripartito alla prima
  lettura valida.
- **Salto** oltre 25 rad/s fra due periodi (stesso limite dell'encoder): errore 9 in controllo.
- **Quiete** (per il riferimento e per B1 da appeso): angolo entro 0,5° da un riferimento per
  1 s. Il rumore del tracciamento deve stare ben sotto 0,5°.
- **Offset**: all'accensione e a ogni B1 da appeso l'angolo fermo diventa esattamente ±180°,
  come la ricalibrazione dell'encoder. Dall'alto l'offset non cambia.
- `PROVA_COLLEGAMENTO 1` (in `main.c`): niente homing e B1 non avvia mai il controllo, il motore
  non si muove; angolo, riferimento e telemetria funzionano. Per le prove da tavolo.
- `HIL_MODE 1`: la USART2 serve all'HIL, l'angolo dal tracciamento non è usato.

## Nodo di prova (senza tracciamento)

`nodo_test` manda angoli sintetici a 200 Hz al posto di `ponte_mocap` e controlla la
telemetria: campioni persi, eta' e ritardo, angolo della scheda = angolo mandato + offset
costante (scarto atteso ~1e-7 rad), reazione ai guasti. Con `durata_s` finisce con l'elenco
dei controlli e `PROVA RIUSCITA` / `PROVA FALLITA`.

```sh
# Firmware con PROVA_COLLEGAMENTO 1 (o carrello libero e driver acceso: dopo il riferimento parte l'homing)
ros2 run pendolo_mocap nodo_test --ros-args -p modo:=appeso -p durata_s:=20.0
ros2 run pendolo_mocap nodo_test --ros-args -p modo:=rotazione -p durata_s:=10.0     # passaggi per ±180°
ros2 run pendolo_mocap nodo_test --ros-args -p modo:=appeso -p guasto:=pausa -p inizio_guasto_s:=5.0 -p durata_s:=10.0
ros2 run pendolo_mocap nodo_test --ros-args -p modo:=appeso -p guasto:=rumore -p durata_guasto_s:=3.0 -p durata_s:=10.0
```

Modi: `appeso`, `oscillazione` (smorzata: la scheda aspetta che si fermi), `rotazione`, `alto`,
`topic` (angolo da `/pendolo/angolo_test`). Guasti: `pausa`, `non_tracciato`, `salto`,
`checksum`, `rumore`. Altri parametri in testa a `nodo_test.py`.

Prova dell'arresto in controllo (errore 12 o 9) con il firmware normale: **senza l'asta o con
l'asta tenuta**, perché la rete muove il carrello in base all'angolo finto. Avviare con
`modo:=appeso guasto:=pausa inizio_guasto_s:=<qualche secondo dopo B1>`, premere B1 dopo
l'homing: il carrello deve fermarsi entro ~70 ms dall'inizio della pausa.

Su questo Mac (senza ROS) gli stessi nodi girano con un rclpy finto contro il firmware
eseguito sul PC: `python3 tests/test_collegamento.py` (ambiente con pyserial).

## Protocollo

USART2 (ST-LINK VCP), 115200 8N1, little-endian, `checksum` = complemento a due della somma
dei byte del corpo (come l'HIL). Riferimento: `ros/pendolo_mocap/pendolo_mocap/protocollo.py`
(usato anche dai test del firmware, che confrontano i byte).

- PC → scheda, 11 byte per campione: `A5 5A | 'M' u8 | stato u8 (bit 0 tracciato) | seq u16 | theta f32 (rad) | checksum`.
- Scheda → PC, 37 byte per periodo: `A5 5A | 'T' u8 | stato u8 (bit 0 controllo, 1 angolo valido, 2 theta inizializzato, 3 carrello inizializzato) | errore u8 | eta_ms u8 | seq u16 | ricevuti u16 | scartati u16 | tick_ms u32 | theta, theta_encoder, stima_theta, stima_x, azione f32 | checksum`.

Occupazione della linea (full duplex, 11 520 byte/s per verso): 2200 byte/s in ricezione
(19 %), 1850 in trasmissione (16 %). Ogni campione impiega circa 1 ms sulla linea.

## Da verificare sul banco: il ritardo

La rete è stata addestrata con un ritardo delle misure di **0-4 ms** (`ritardo_max_s` 0,012 a
intensità 0,35). L'encoder era praticamente istantaneo; il tracciamento no: elaborazione del
sistema (tipicamente 5-15 ms, dipende dal sistema), attesa del campione (0-5 ms a 200 Hz),
ROS, USB e linea (2-3 ms). `eta_ms` e `ritardo_ms` misurano solo il tratto nodo → scheda.
Il ritardo totale si misura con l'encoder ancora montato: sfasamento fra `theta` e
`theta_encoder` su `/pendolo/stm32` durante un'oscillazione libera. Se supera di molto i 4 ms
e lo swing-up o l'equilibrio peggiorano rispetto all'encoder, la strada è riaddestrare con
`ritardo_max_s` adeguato (non cambiare il firmware).
