# Pendolo inverso su carrello — controllo con reinforcement learning su STM32

Swing-up ed equilibrio di un pendolo su carrello mosso da un motore stepper, con una rete
neurale addestrata in simulazione (SAC + MuJoCo) ed eseguita su una NUCLEO-F446RE (X-CUBE-AI).

## Struttura

| Cartella | Contenuto |
|---|---|
| [`cart_pendolo_velocita_rl/`](cart_pendolo_velocita_rl/LEGGIMI.md) | ambiente di simulazione, addestramento, valutazione ed esportazione nel firmware; rete consegnata in `modelli_asta20_basevecchia_storia1s_20s/` |
| [`Controllo_SwingUp_Asta20_Storia_BaseVecchia/`](Controllo_SwingUp_Asta20_Storia_BaseVecchia/LEGGIMI.md) | progetto STM32CubeIDE del firmware: encoder, stimatore dello stato, rete (132 ingressi), comando dello stepper, homing, protocollo HIL |

## Uso rapido

Firmware (asta a 20 cm dal centro del perno al capo corto):

1. STM32CubeIDE → *File → Import → Existing Projects into Workspace* →
   `Controllo_SwingUp_Asta20_Storia_BaseVecchia`.
2. *Project → Build Project*, poi *Run*. Non rigenerare il codice da CubeMX (vedi il `LEGGIMI.md` del firmware).
3. Pendolo appeso e fermo, premere **B1**: swing-up ed equilibrio. B1 ferma tutto.

Addestramento (dalla cartella `cart_pendolo_velocita_rl`):

```sh
./Setup.command                      # ambiente Python
./Avvia_RL.command                   # rete consegnata in simulazione
./Addestra.command                   # nuovo addestramento con config.toml
python esporta_firmware.py --model modelli_riaddestramento/migliore.zip   # nel firmware
```

## Test

```sh
cd cart_pendolo_velocita_rl
python -m unittest test_configurazione test_disturbi test_periodo test_stima test_storia test_versione
cd ..
python Controllo_SwingUp_Asta20_Storia_BaseVecchia/tests/test_controllo.py
python Controllo_SwingUp_Asta20_Storia_BaseVecchia/tests/test_stima.py
python Controllo_SwingUp_Asta20_Storia_BaseVecchia/tests/test_hil.py
python Controllo_SwingUp_Asta20_Storia_BaseVecchia/tests/test_candidati.py
```

I test del firmware compilano il codice C reale con la HAL simulata e lo confrontano con la
simulazione Python (rampa, inversioni, storia, stimatore, pesi della rete).

## Rami

- `main`: versione consegnata (questa).
- `archivio-completo`: tutte le versioni precedenti di firmware e reti, configurazioni delle
  prove, variante con motion capture/ROS.
