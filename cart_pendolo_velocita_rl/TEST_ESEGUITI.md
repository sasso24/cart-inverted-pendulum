# Baseline vecchia + perno randomizzato largo, da zero — 24 ottobre 2026

Configurazione delle reti del 21-23/10 (`cart_pendolo_asta20.xml` con perno 0,01 N·m·s, stimatore
con k_d 0,403, stessa ricompensa) + perno randomizzato per episodio su un intervallo largo:
viscoso log-uniforme 0,0003-0,015 N·m·s, coulombiano 0-0,006 N·m (comprende la stima a memoria
del banco, 0,0095 + 0,006), aria 0-6,6e-4 N·m·s². Da zero, 2 M passi senza storia (seed 42, 7, 3),
2,5 M con storia (`config_asta20_basevecchia_*.toml`).

Senza storia la selezione automatica non funziona: in nominale (perno dell'XML, 0,01) nessuna
rete sale in tempo, il minimo tra nominale e disturbato resta 0 e `migliore.zip` è casuale
(s3: 140 k passi). Valutati sia `migliore.zip` sia `ultimo.zip` con `prova_smorzamento.py`
(20 semi per valore, disturbi):

| Rete | 0,0003-0,005, attrito 0,0005 (su 80) | 0,0003-0,005, senza coulombiano (su 80) | 0,01 + 0,0005 | stima banco 0,0095 + 0,006 |
|---|---:|---:|---:|---:|
| **basevecchia s7 `migliore`** (1,6 M) | 80 | 60 | 0 | 0 |
| **basevecchia s3 `ultimo`** (2 M) | 73 | 57 | 0 | 0 |
| basevecchia s42 `ultimo` | 74 | 50 | 0 | 0 |
| basevecchia s7 `ultimo` | 71 | | 0 | 0 |
| basevecchia s42 `migliore` (1,32 M) | 52 | | 0 | 0 |
| basevecchia s3 `migliore` (140 k) | 30 | | 0 | 0 |
| **basevecchia storia `migliore`** (1,78 M) | **80** | **80** | 0 (su a 13,1 s) | **16** (nominale 18) |
| basevecchia storia `ultimo` (2,5 M) | 80 | 68 | 1 | 11 (nominale 14) |
| rif. storia 23/10 | 11 | 3 | | 17 (nominale 18) |
| rif. perno storia (24/10, baseline 0,002) | 80 | 79 | | 0 (nominale 3) |

La rete con storia è l'unica che regge sia un perno che frena poco sia la stima del banco.
Le reti senza storia non arrivano in alto entro 20 s con perni molto frenati (meno di un
episodio su cinque in addestramento ha viscoso ≥ 0,007).

Firmware (copie, i progetti esistenti non cambiano): `Controllo_SwingUp_Asta20_BaseVecchia`
(rete 1 = s7 `migliore`, 2 = s3 `ultimo`, 0 = copia della 1) e `Controllo_SwingUp_Asta20_Storia_BaseVecchia`
(una rete). Test `test_candidati`, `test_controllo`, `test_hil` OK (HIL senza storia: 2 swing-up su 3
nei semi del test, con perno estratto a caso; con storia 3/3). Verifica ONNX di riferimento fallita per
1,4e-5 (tolleranza 1e-5) su una rete senza storia: riguarda solo il file ONNX.

---

# Perno randomizzato e stimatore con k_d coerente — 24 ottobre 2026

Ricetta delle reti `modelli_asta20_20s*` / `storia1s_20s` (episodi 20 s, stimatore
dell'offset, sensori 16/10) con: XML `cart_pendolo_asta20_perno.xml` (perno nominale
0,002 N·m·s + 0,001 N·m), perno randomizzato per episodio (viscoso log-uniforme
0,0003-0,012 N·m·s, coulombiano 0-0,003 N·m, aria 0-6,6e-4 N·m·s²), stimatore con
`k_d` 0,0807 (prima 0,403). 2 M passi senza storia, 2,5 M con storia.

| Addestramento | miglior checkpoint | note |
|---|---|---|
| `modelli_asta20_perno_20s_s3` (seed 3) | 1,94 M, selezione 6/6 + 6/6 | **`Controllo_SwingUp_Asta20_Perno`, rete 1 (e 0)** |
| `modelli_asta20_perno_20s_s7` (seed 7) | 1,88 M, 6/6 + 6/6 | **`Controllo_SwingUp_Asta20_Perno`, rete 2** |
| `modelli_asta20_perno_20s` (seed 42) | 2,36 M (ripreso +1 M da 2 M: fino a 2 M mai sopra 5/6) | scartata |
| `modelli_asta20_perno_storia1s_20s` (seed 42) | 1,84 M, 6/6 + 6/6 | **`Controllo_SwingUp_Asta20_Storia_Perno`**; dopo 1,9 M crollo a 0% fino alla fine |

**Sweep dello smorzamento** (`prova_smorzamento.py`, 20 semi, 20 s; smorzamento del perno
fisso, stimatore della rete). Successi con disturbi, senza attrito coulombiano né aria
(caso più difficile: in addestramento c'è sempre un po' di attrito):

| damping perno (N·m·s) | 0,012 | 0,01 | 0,005 | 0,0025 | 0,001 | 0,0003 |
|---|---:|---:|---:|---:|---:|---:|
| `modelli_asta20_storia1s_20s` (23/10) | 1 | 20 | 3 | 0 | 0 | 0 |
| `modelli_asta20_20s` (21/10) | 0 | 13 | 2 | 0 | 0 | 2 |
| **perno s3** | 0 | 1 | 20 | 20 | 20 | 17 |
| **perno s7** | 0 | 0 | 20 | 20 | 17 | 1 |
| perno s42 | 0 | 3 | 19 | 20 | 9 | 4 |
| **perno storia** | 0 | 11 | 20 | 20 | 20 | 19 |

Passaggi sopra la verticale (media, con disturbi) da 0,005 a 0,0003: reti del 21-23/10
5-16; nuove s3 e storia 0-1,6 (s7 6,1 a 0,0003). Primo arrivo in alto 6-7,5 s (prima 9 s con 0,01).
In nominale (stato vero) le nuove s3, s7, storia: 20/20 da 0,005 a 0,0003, 16-19/20 a 0,01.

Con attrito realistico (`--attrito 0.0005 --aria 0.00033`), disturbi, 0,005 / 0,0025 / 0,001 / 0,0003:
s3 20/20/20/20, s7 19/20/20/19, storia 20/20/20/20, s42 19/20/20/19; rete storia del 23/10 8/0/1/2.

**Limite**: con perno a 0,01-0,012 e disturbi le nuove reti spesso non arrivano in alto in
20 s (le vecchie sì). Il banco (salita facile, pendolo che gira) indica un perno meno
smorzato, ma va misurato.

**Stima del perno a memoria (24/10, dopo `prova_perno.py`)**: smorzamento 0,0095 N·m·s,
attrito 0,006 N·m, aria 1e-5 (da 90° si ferma in ~11 s, 6,5 oscillazioni; XML vecchio 25 s,
nuovo nominale 55 s). Con questo perno (`prova_smorzamento.py --smorzamenti 0.00954
--attrito 0.00596 --aria 0.00001`, nominale / disturbi su 20): storia 23/10 18/17, 20s 21/10 3/3,
perno s3 2/0, s7 0/0, storia 3/0; nessuna supera la verticale. Se la stima è giusta il
sintomo del banco (pendolo che gira) non viene dal perno. Progetti firmware vecchi riportati
alle reti del 21 e 23/10 (per provarle con un altro encoder), nuove reti nei progetti `_Perno`.

`confronta.py`, 40 semi, episodi da 20 s (successo = su da 12 s alla fine; tra parentesi
"arrivato su e mai caduto"). Le condizioni disturbate usano il perno randomizzato della rete:

| Rete | nominale | disturbi | sensori 16/10 | offset 4° | inversione | aggancio | inversioni/s nom. / dist. |
|---|---:|---:|---:|---:|---:|---:|---:|
| perno s3 | 40 (40) | 35 (40) | 36 (40) | 34 (40) | 35 (40) | 36 (36) | 10,6 / 7,0 |
| perno s7 | 40 (40) | 33 (40) | 33 (40) | 32 (39) | 33 (40) | 34 (34) | 28,6 / 7,1 |
| perno storia | 40 (40) | 36 (40) | 35 (40) | 35 (40) | 36 (40) | 32 (32) | 26,0 / 13,0 |

Firmware: test `test_candidati`, `test_controllo`, `test_hil` (3/3 swing-up con il codice C)
OK in entrambi i progetti. L'ONNX di riferimento di s7 non è stato scritto: la verifica di
`esporta_onnx.py` fallisce per 1,3e-5 su 1 ingresso di 5100 (tolleranza 1e-5). Riguarda solo
il file ONNX: i pesi del firmware sono scritti prima e sono identici al checkpoint.

---

# Storia degli ingressi, asta 0,20 m — 23 ottobre 2026

Nuova sezione `[storia]` (`configurazione.py`, `storia.py`): per ogni ritardo k la rete
riceve i sei ingressi di k periodi fa e l'azione decisa allora; vuota = sei ingressi come
prima. Ricetta delle reti `modelli_asta20_20s*` (episodi 20 s, stimatore dell'offset,
sensori 16/10), 2 M passi, seed 42 e 7:
*1 s ridotta* = ritardi 1..10 e 15, 20, ..., 50 (132 ingressi); *0,2 s* = ritardi 1..10 (76).

Le reti con storia imparano molto prima (ricompensa 320-470 a 200 k passi, le due con
seed 7 gia al 67-83% di successi; senza storia ~126 a 500 k). Confronto `confronta.py --durata 60`, 40 semi:

| Rete | nominale | disturbi | sensori 16/10 | offset 4° | inversione | aggancio | totale (successo / su) | inversioni/s nom. |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `modelli_asta20_storia1s_20s` (firmware, rete 0) | 40/40 | 40/40 | 39/40 | 31/40 | 40/40 | 40/40 | 230 / 240 | 16,8 |
| `modelli_asta20_storia1s_20s_s7` (firmware, rete 1) | 40/40 | 39/40 | 39/40 | 31/39 | 39/40 | 40/40 | 228 / 239 | 35,2 |
| `modelli_asta20_storia02_20s_s7` | 40/40 | 40/40 | 37/40 | 34/40 | 40/40 | 40/40 | 231 / 240 | 40,8 |
| `modelli_asta20_storia02_20s` | 40/40 | 37/40 | 38/40 | 27/40 | 37/40 | 40/40 | 219 / 240 | 21,8 |
| rif. `modelli_asta20_20s` (senza storia) | 38/40 | 31/40 | 27/40 | 18/40 | 31/40 | 40/40 | 185 / 240 | 11,5 |
| rif. 0,16 m `modelli_60s_1200k` | 40/40 | 39/40 | 40/40 | 28/38 | 39/40 | 40/40 | 226 / 238 | 19,8 |

Salita mediana con disturbi 9,5-10,1 s (11,2 s senza storia).

- Codice: `storia.py` (mixin sopra l'ambiente nominale e quello disturbato, attivo solo
  con `[storia].ritardi`), `crea_ambiente`, firma con `observation_size`, `storia.py` tra
  i file del contratto; `esporta_onnx.py` per n ingressi (tolleranza della verifica
  PyTorch/ONNX scalata con la radice di n/6: invariata per 6 ingressi);
  `esporta_firmware.py` scrive `CFG_STORIA_*` e rigenera `X-CUBE-AI/App` con `stedgeai`
  se gli ingressi cambiano; `hil.py` confronta i sei ingressi attuali.
- Reti esistenti: con il codice nuovo stesse ricompense e durate su nominale e disturbato
  (`modelli_asta20_20s`, 4+4 episodi di `confronta.py` identici), ONNX esportato identico
  byte per byte; compare l'avviso "codice cambiato".
- `python -m unittest test_storia test_versione test_disturbi test_stima test_configurazione
  test_periodo`: 47 test, tutti OK tranne `test_guadagni_equivalenti` (manca `scipy.optimize`
  nel venv, indipendente da queste modifiche). `test_storia`: ingressi e azioni ai ritardi
  giusti, reset, nessun effetto su fisica e numeri casuali.
- `stedgeai generate` (X-CUBE-AI 10.1.0) rigenera la rete a 6 ingressi identica a quella
  del progetto (cambiano solo le firme del modello).
- Firmware `Controllo_SwingUp_Asta20_Storia` (copia di `Controllo_SwingUp_Asta20`, che resta
  intatto): reti 0-1 qui sopra, 132-128-128-1, 33 931 MACC. gcc 12.3 di CubeIDE: 0 warning
  nel codice del progetto, 307 KB di flash (anche `HIL_MODE 1`). `test_candidati` OK,
  `test_controllo` OK (anche storia: ritardi, azioni, riavvio), `test_stima` OK (sostituto
  numpy di `place_poles`), `test_hil` 3/3 swing-up con la rete a 132 ingressi.
- Non provato: scheda e banco; tempo di inferenza reale (`rete_durata_ms`).

Dati: `risultati/confronto_asta20_storia*.json`, log in `log_asta/`.

---

# Asta spostata nel mozzo: braccio corto 0,30-0,35 m e 0,20 m — 21 ottobre 2026

Stessa asta (0,98 m, acciaio 5 mm) con un braccio corto più lungo: XML
`cart_pendolo_asta20/30/33/35.xml` (cambiano solo i due `fromto` dell'asta),
k_g / k_a / k_d da `stima.coefficienti_da_modello`, guadagni da
`stima.guadagni_con_offset` (calcolati in numpy: scipy manca nel venv, stessi valori
di prima sulla geometria storica fino a 1e-12). Ricette: `config_60s.toml` con
episodi da 60 s (2 M passi) o da 20 s (1,5 M passi), stimatore dell'offset in addestramento.

| braccio corto | k_g (1/s²) | k_a (rad/m) | k_d (1/s) | inerzia (g·m²) | m·d (g·m) | energia max pompaggio |
|---|---:|---:|---:|---:|---:|---:|
| 0,16 (storico) | 17,13 | 1,746 | 0,350 | 28,5 | 49,9 | 10,7 |
| 0,20 | 17,33 | 1,767 | 0,403 | 24,8 | 43,8 | 1,20 |
| 0,30 | 16,05 | 1,636 | 0,570 | 17,6 | 28,7 | 0,30 (bang-bang) |
| 0,33 | 14,85 | 1,514 | 0,627 | 16,0 | 24,2 | 0,02 (bang-bang) |
| 0,35 | 13,78 | 1,405 | 0,664 | 15,1 | 21,2 | −0,17 (bang-bang) |

- **0,30 / 0,33 / 0,35 m**: 9 addestramenti (60 s seed 42 e 7, 20 s seed 42), fermati a
  0,7-1,0 M passi con 0% di successi: le reti pompano fino a ~110° e non salgono.
  Causa fisica: lo smorzamento del perno (0,01 N·m·s nell'XML, non identificato) pesa
  1,6-1,9 volte di più con l'inerzia minore e k_a cala, quindi il carrello a 0,48 m/s
  non dà abbastanza energia (energia massima < 1 anche col pompaggio ideale). Con
  smorzamento 0,0025 o carrello a 0,72 m/s lo swing-up a 0,30-0,33 m tornerebbe possibile.
- **0,20 m** (scelto, margine di energia 20%): 5 addestramenti. Confronto
  `confronta.py --durata 60`, 40 semi, "successo / su e mai caduta":

| Rete | nominale | disturbi | sensori 16/10 | offset 4° | inversione | aggancio | totale "su" |
|---|---:|---:|---:|---:|---:|---:|---:|
| `modelli_asta20_20s` (rete 0) | 38/40 | 31/40 | 27/40 | 18/40 | 31/40 | 40/40 | 240/240 |
| `modelli_asta20_20s_s7` (rete 1) | 37/40 | 30/40 | 30/40 | 8/40 | 30/40 | 40/40 | 240/240 |
| `modelli_asta20_60s_s3` (rete 2) | 40/40 | 23/40 | 18/38 | 3/39 | 23/40 | 39/39 | 236/240 |
| `modelli_asta20_60s` | 40/40 | 34/38 | 28/39 | 13/36 | 34/38 | 37/37 | 228/240 |
| `modelli_asta20_60s_s7` | 13/40 | 11/34 | 11/36 | 0/31 | 11/34 | 35/35 | 210/240 |
| rif. 0,16 m `modelli_60s_1200k` | 40/40 | 39/40 | 40/40 | 28/38 | 39/40 | 40/40 | 238/240 |

  La salita a 0,20 m dura ~11 s (mediana) contro ~10 s: il criterio "su a 12 s" penalizza.
- Firmware `Controllo_SwingUp_Asta20` (copia di `Controllo_SwingUp`, che resta intatto):
  reti 0-2 qui sopra. Compilato con il gcc 12.3 di STM32CubeIDE e le opzioni del
  progetto: 0 warning nel codice del progetto, 247 KB di flash (anche `HIL_MODE 1`).
  `test_candidati` OK, `test_controllo` OK, `test_stima` OK (1,2e-6, eseguito con un
  sostituto numpy di `scipy.signal.place_poles`), `test_hil` 3/3 swing-up,
  diff. ingressi 9e-7, azione 2e-6.
- Non provato: scheda e banco con l'asta spostata; smorzamento reale del perno.

Dati: `risultati/confronto_asta20_*.json`, log in `log_asta/`.

---

# Episodi da 60 s, partenze casuali e firmware con più reti — 17 ottobre 2026

Tutte le reti a 20 ms con lo stimatore dell'offset (anche in addestramento),
sensori 16/10 (offset ±3°, conteggi persi ≤0,3%), 2 M passi.

| Addestramento | esito |
|---|---|
| episodi 60 s, senza partenze casuali (`config_60s.toml`) | migliore a 1,2 M passi (100% nominale e disturbato sui 6 semi di selezione), poi oscilla |
| episodi 60 s, 30% partenze casuali (`config_60s_casuali.toml`) | crollato dopo ~750 k passi (ricompensa negativa per >300 k), fermato |
| episodi 60 s, 15% partenze casuali, seed 7 (`config_60s_casuali15.toml`) | crolli dopo ~450 k passi |

Confronto (`confronta.py --durata 60`, 40 semi; "su" = arrivato in alto entro
20 s e in equilibrio fino alla fine):

| Rete | nominale | disturbi | sensori 16/10 | offset 4° | aggancio | inversioni/s (nom. / dist.) |
|---|---:|---:|---:|---:|---:|---:|
| 14/10 + stimatore offset | 40 (su 40) | 39 (40) | 37 (40) | 38 (40) | 40 (40) | 41 / 11 |
| 60 s, 1,2 M passi (`modelli_60s_1200k`) | 40 (40) | 39 (40) | 40 (40) | 28 (38) | 40 (40) | 20 / 8 |
| 60 s, 1,0 M passi | 34 (40) | 29 (40) | 23 (40) | 9 (40) | 39 (39) | 20 / 8 |

- La durata degli episodi in valutazione non cambia niente per la rete 14/10 +
  offset (stessi risultati a 20 s e a 60 s): in simulazione, una volta su, non cade.
- La rete a 60 s sale più lentamente (13-14 s contro 9-10 s): il criterio "su
  entro 12 s" la penalizza, ma una volta su non cade quasi mai; inverte il
  verso del motore la metà delle volte.
- Le partenze casuali, in questa forma, rendono instabile l'addestramento
  (due semi su due). Restano utili come prova (`aggancio`).
- Firmware `Controllo_SwingUp` con tre reti selezionabili (`rete_scelta`):
  compilato con arm-none-eabi-gcc 13.2 con le opzioni di CubeIDE (0 warning nel
  codice del progetto, 248 KB di flash su 512); test PC: `test_controllo`
  (anche cambio di rete), `test_stima`, `test_candidati` (pesi e guadagni
  identici ai checkpoint), `test_hil` 3/3.

---

# Stimatore dell'offset e prove a 10 ms — 16 ottobre 2026

Confronto con `confronta.py`, 40 semi (2000-2039), stesse condizioni per tutte le reti:
*disturbi* = profilo della rete + guasti sensori del 14/10 (offset ±2°, conteggi
persi ≤0,2%); *sensori 16/10* = offset ±3°, ≤0,3%; *offset 4°* = encoder spostato
di 4° fissi; *inversione* = rampa ferma 2 ms al cambio di verso come il firmware;
*aggancio* = partenza da stato casuale, successo se in equilibrio negli ultimi 8 s.

| Rete | nominale | disturbi | sensori 16/10 | offset 4° | inversione | aggancio | inversioni/s (nom.) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 14/10, 20 ms (sulla scheda fino al 16/10) | 40 | 29 | 18 | 0 | 29 | 24 | 22 |
| **14/10 + stimatore dell'offset** (firmware dal 16/10) | **40** | **39** | **37** | **38** | **39** | **40** | 22 |
| 10 ms, seed 7, 2,68 M passi | 40 | 11 | 12 | 2 | 11 | 13 | 15 |
| 10 ms + stimatore dell'offset, seed 42, 0,96 M | 35 | 25 | 26 | 11 | 25 | 40 | 23 |

- Lo stimatore dell'offset da solo, senza riaddestrare, porta la rete del 14/10
  quasi a 40/40 in tutte le condizioni. Test `test_periodo.OffsetTest`: con 4°
  di offset l'offset stimato a fine episodio è entro 1° da quello vero.
- **10 ms non conviene, per ora**: addestramenti a 10 ms (gamma 0,995,
  ricompensa riscalata, 3 M passi) instabili, con crolli della ricompensa
  anche dopo il 100% di successi; le reti migliori restano sotto quella del
  14/10. Prova a 20 ms con sensori 16/10 e partenze casuali ma stimatore a due
  stati: 0% di successi in 1,5 M passi, quindi il problema principale era
  l'offset dell'encoder, non il periodo.
- Firmware `Controllo_SwingUp` (test su PC): `test_controllo.py` OK (rampa C =
  `stepper.py` su 2000 passi casuali), `test_stima.py` OK (C = Python anche con
  offset, differenza < 2e-6), `test_hil.py` 3/3 swing-up, diff. ingressi 9e-7.
  Da compilare in STM32CubeIDE e provare sul banco.

Dati: `risultati/confronto_swingup.json`, `confronto_swingup_offset.json`,
`confronto_10ms.json`, `confronto_offset_960k.json`.

---

# Swing-up da -180° e configurazione unica — 14 ottobre 2026

**Rete** `modelli_swingup/migliore.zip` (6-128-128-1, SAC, seed 42, scelta a
1,02 M passi su 1,5 M; `config.toml` = configurazione salvata con la rete).
Addestramento con disturbi (intensità 0,35, ritardo fino a 4 ms) e guasti
sensori (offset encoder ±2°, conteggi persi fino a 0,2%, 0,5 impulsi spuri/s
da 1-4 conteggi, passi persi fino al 2%), rumore 5-10 mm / 0,5-1°.

**Valutazione** (`valuta.py`, 40 semi 2000-2039 mai usati in addestramento,
episodi da 20 s, partenza appesa ±3°; successo = in equilibrio senza
interruzioni da 12 s alla fine):

| | nominale | disturbi + guasti sensori |
|---|---:|---:|
| Successi | 40/40 | 27/40 |
| In equilibrio alla fine dell'episodio | 40/40 | 28/40 |
| Tempo di salita (medio / max) | 9,4 / 10,2 s | 9,6 / 11,6 s |
| Corsa massima | 0,20 m | 0,37 m (limite 0,40) |
| Arresti per limite di corsa | 0 | 0 |

Stessi 40 semi, rete invariata, un solo tipo di guasto per volta:
senza guasti sensori 40/40; solo offset ±2° 32/40 successi (37/40 in
equilibrio alla fine); guasti senza offset 36/40 (37/40). L'offset iniziale
pesa di più: nel firmware viene azzerato a ogni avvio (ricalibrazione a -180°).

Un controllore euristico di pompaggio dell'energia sullo stesso modello sale
in 8-11 s: con 0,48 m/s di velocità massima del carrello la salita non può
essere molto più rapida. Nelle cadute il carrello deriva verso un lato
(offset/deriva dell'angolo) e la rete spesso fa un giro completo e riprende
il pendolo. Durante l'equilibrio il comando cambia ancora spesso verso
(`peso_inversione = 0,02` non basta).

**Altre prove**

- Codice nuovo con i vecchi checkpoint: `modelli_disturbi/migliore.zip`,
  nominale e disturbato, 1.800 passi: osservazioni, stato e ricompense
  identici bit per bit al codice del 9 ottobre.
- `python -m unittest test_versione test_disturbi test_stima test_configurazione`:
  35 test OK. `verifica.py` fallisce sulla manovra manuale da 10 cm come già
  il codice del 9 ottobre (minimo 3.200 Hz).
- `esporta_onnx.py`: 5.100 ingressi, errore massimo PyTorch/ONNX 5,2e-6.
- Firmware `Controllo_SwingUp`: compilato con arm-none-eabi-gcc 13.2
  (anche `HIL_MODE 1`), 0 warning; text 106.944 B.
  `tests/test_controllo.py` (codice USER CODE reale, HAL simulata: ricalibrazione
  all'avvio, salti encoder, limiti di corsa, B1, fault, rampa) OK;
  `tests/test_stima.py` OK (1,4e-6); `tests/test_hil.py` OK (protocollo e pesi,
  diff ingressi 1,4e-6, diff azione 2,3e-6; swing-up 2/3).
- `Prova_controllo/tests/test_hil.py` e `test_stima.py` passano ancora col
  codice Python nuovo.
- Ricalibrazione condizionata (sera del 14/10): a B1 l'angolo torna a -180°
  solo se quello letto è entro ±20° da -180° (e l'encoder è fermo); entro ±10°
  da 0° la rete parte con l'equilibrio senza ricalibrare; altrove avvio
  rifiutato. `tests/test_controllo.py` OK (anche le soglie), firmware
  ricompilato 0 warning (text 107.072 B). Stessa rete, partenza dall'alto
  ±3° in simulazione: 20/20 in equilibrio per 20 s con sensori ideali,
  16/20 con disturbi e guasti.
- Non provato: scheda reale, motore, encoder sul banco.

---

# Hardware-in-the-loop — 9 ottobre 2026

- Firmware: `HIL_MODE` in `main.c` (default 0). Con 1 la scheda riceve le misure
  simulate via USART2, esegue `stima.c` e la rete X-CUBE-AI, risponde col comando.
  Compilato con arm-none-eabi-gcc in entrambe le modalità, 0 warning.
- PC: `hil.py` / `HIL.command`, istruzioni in `Prova_controllo/HIL.md`.
- Senza scheda (`Prova_controllo/tests/test_hil.py`): codice HIL reale di main.c
  su host, UART su pseudo-terminale, rete come MLP C con i pesi del firmware.
  3/3 episodi stabili, diff ingressi 1,6e-7, diff azione 3,9e-7.
- Da fare sulla scheda: tempi reali, runtime ST, seriale ST-LINK.

# Stimatore nel firmware STM32 — 9 ottobre 2026

- `Prova_controllo/Core/Src/stima.c`: porting di `stima.py`, predizione in
  SysTick ogni 2 ms, correzione in `Rete_Process` ogni 20 ms. Rete aggiornata
  ai pesi di `modelli_disturbi/migliore.zip` (`aggiorna_pesi_xcubeai.py`).
- C contro Python: 20 sequenze da 60 s con wrap ±pi, differenza massima 1,4e-6.
- Catena del firmware in simulazione (stima.c via ctypes + pesi letti da
  `network_data_params.c`, float32): 60/60 episodi da 60 s stabili
  (disturbi 0,25, intensità 1, rumore massimo 10 mm / 1°), angolo max 6,6°.
- `test_controllo.py` aggiornato e superato (ingressi stimati, integrazione
  della velocità STEP, aggancio della posizione a controllo fermo).
- Compilazione e link del firmware completo con arm-none-eabi-gcc 13.2
  (Cortex-M4 hard-float, runtime ST): 0 warning, 97,6 kB di flash.
- Non eseguiti: caricamento sulla scheda, Validate X-CUBE-AI, prova sul banco.
  `tests/test_control_button.py` non compila già nella versione precedente
  (manca il mock della rete X-CUBE-AI).

# SAC dall’alto con rumore dei sensori — 1 ottobre 2026

- Rimosso il controllo LQR (`equilibrio.py`, test, launcher, profilo e documento).
- Rumore: posizione 5–10 mm, encoder 0,5–1° (1 sigma per lettura a 50 Hz,
  estratto per episodio), più quantizzazione 2400 conteggi/giro e offset ±0,125°.
  Disturbi fisici invariati, `intensita = 0.25`.
- Nuovo stimatore `stima.py`: senza di esso anche un controllore ideale
  cadeva in 1–2 s con questi livelli di rumore (velocità derivate inutilizzabili).
- Ricompensa: termine di centratura del carrello più stretto (scala 15 cm)
  e piccola penalità sulle variazioni del comando. Con la vecchia ricompensa
  la rete stava in alto, ma nelle prove da 60 s il carrello derivava fino al finecorsa.
- Addestramento: 400k passi da zero con disturbi, poi 250k passi di rifinitura
  con la nuova ricompensa. Rete scelta: `modelli_disturbi/migliore.zip`.
- `Valuta.command` (semi 2000–2019, 20 s): 20/20 disturbati, 20/20 nominali.
  Rapporto: `risultati/valutazione_disturbi.json`.
- Prova lunga, semi 4000–4019, 60 s per episodio, 100/100 interamente stabili:
  nominale, disturbi 0,25, disturbi a intensità 1, rumore fisso al massimo
  (10 mm, 1°), spinte di ±1 N per 100 ms. Angolo massimo 4,5° (partenza fino
  a 2,9°), carrello entro 15 cm dal centro.
- 25 test automatici superati: `python -m unittest test_versione test_disturbi test_stima`.
- `verifica.py` si ferma nella manovra manuale con le frecce: lo fa anche la
  versione precedente non modificata, quindi non dipende da queste modifiche.
  Prove eseguite con MuJoCo 3.14 (requirements: 3.12).
- Nessuna prova sul banco. Lo stimatore va portato sul firmware (vedi `TODO_STM32.md`).

# Disturbi e vibrazioni - 29 settembre 2026

- 21 test automatici superati: i 14 esistenti e 7 nuovi per configurazioni,
  determinismo, assenza di deriva dei parametri fra reset, modi oscillanti e
  decadimento, integrazione esatta, ritardi, limiti, arresti, separazione fra
  rumore di misura e fisica, compatibilita Gymnasium e contratti checkpoint.
- Il test storico della rampa ora imposta esplicitamente 64.000 Hz/s nella
  propria fixture: assumeva questa rampa anche dopo che il TOML era passato
  a 1.600.000 Hz/s. Nessun cambiamento al parametro hardware.
- Prova SAC temporanea di 3.104 decisioni (incluse ottimizzazioni), valutazione
  doppia nominale/disturbata, salvataggio, ripresa per altre 8 decisioni e
  simulazione senza finestra completati. Non e un addestramento completo.
- La rete esistente `modelli/migliore.zip` e stata valutata sui semi 2000–2019:
  20/20 successi nominali e 20/20 con il profilo `disturbi.toml` a intensita 0,35.
  Ricompense medie rispettivamente 607,98 e 601,90; durata media della migliore
  sequenza stabile 9,287 s e 9,09 s. Nessun finecorsa nelle due suite.
  Rapporto: `risultati/stress_rete_esistente.json`.
- Questi risultati usano il criterio storico di almeno 3 s consecutivi di
  equilibrio. Il profilo iniziale e lieve e non identificato sul banco:
  non dimostra robustezza alle risonanze reali e non riproduce il fallimento
  osservato sull'hardware. Frequenze e ampiezze restano configurabili.
- I checkpoint nominali esistenti superano ancora il controllo di compatibilita.
  XML, ambiente nominale, TOML hardware e firmware non sono stati modificati.
- Nessuna prova hardware o verifica visiva della finestra. La verifica geometrica
  e la manovra manuale storica non sono state rieseguite per questa modifica;
  resta la limitazione della manovra riportata sotto.

# Rampa di frequenza - 28 settembre 2026

- 14 test automatici superati, inclusi salita, frenata, inversione, assenza
  di overshoot, reset, finecorsa e aggiornamento della rampa a ogni passo fisico.
- Verificata la distinzione fra frequenza richiesta e applicata, con stato della
  rampa nel sesto ingresso della rete; Gymnasium e Stable-Baselines3 accettano l'ambiente.
- 1.805 configurazioni geometriche senza collisioni interne alla corsa.
- Prova SAC di 128 decisioni con aggiornamenti della rete, salvataggio,
  caricamento, inferenza e ripresa per altre 16 decisioni riuscita. File di prova
  temporanei; i modelli dell'utente non sono stati modificati.
- Checkpoint precedenti senza rampa rifiutati correttamente.
- La verifica completa `verifica.py` si ferma sulla manovra manuale di 10 cm:
  con il minimo attuale di 3.200 Hz la manovra va in timeout nei due versi.
  Il confronto con il codice precedente alla modifica e gli stessi parametri
  conferma che il problema era gia presente. Con minimo zero, la manovra con
  rampa termina nei due versi entro 0,5 mm dal bersaglio. Il minimo dell'utente
  resta invariato; il rapporto JSON storico non e stato rigenerato.
- Nessuna verifica visiva del viewer, prova hardware o valutazione di una rete
  addestrata con la rampa. Il test breve verifica soltanto la pipeline.

# Verifiche della versione senza rampa — 22 settembre 2026

- Otto test automatici superati: conversioni, limiti, configurazioni invalide,
  microstepping, Gymnasium, accoppiamento del pendolo, arresto e finecorsa,
  episodio completo e reset.
- 1.805 configurazioni geometriche senza collisioni interne alla corsa.
- Manovre manuali di 10 cm completate nei due versi; viewer continuo
  verificato senza aprire la finestra.
- Geometria e opzioni XML confrontate con l'originale: cambia solo l'attuatore.
- Addestramento SAC di prova: 3.200 decisioni, inclusi aggiornamenti della rete,
  selezione del checkpoint, salvataggio e replay buffer.
- Ripresa del checkpoint per altre 64 decisioni riuscita.
- Simulazione headless, valutazione su due semi, generazione CSV e PNG riuscite.
- Vecchi checkpoint a sei ingressi rifiutati correttamente.
- Sintassi dei launcher macOS controllata.

Le prove della pipeline usano Python 3.12 e l'ambiente già presente nel progetto
originale (MuJoCo 3.12.0, Stable-Baselines3 2.9.0, PyTorch 2.14.0).
Gli avvisi Gymnasium sui limiti infiniti delle osservazioni sono attesi.

La rete del test breve non ha imparato lo swing-up: zero successi nelle due
prove. Non viene consegnata come rete addestrata. I checkpoint di test sono
separati nella cartella temporanea `/private/tmp/cart-hz-pipeline-20260922`.
Occorre avviare l'addestramento completo nella nuova cartella e valutarne i risultati.
La finestra interattiva non è stata verificata visivamente; sono stati controllati
il codice del viewer e i suoi comandi manuali tramite simulazione senza GUI.
Nessuna prova hardware eseguita.
