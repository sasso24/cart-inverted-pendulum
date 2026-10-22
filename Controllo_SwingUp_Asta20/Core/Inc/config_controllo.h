/* GENERATO da cart_pendolo_velocita_rl/esporta_firmware.py: NON modificare a mano.
 * Rete: modelli_asta20_20s/migliore.zip (2026-10-21)
 * sha256 891ad412de1dd9de878e3c1c36ea5e0120cc2d04e90d904219eec2ef2dd967f7
 * Altre reti nello stesso firmware: Core/Src/reti_candidate.c.
 * Per cambiare un valore: config.toml -> nuovo addestramento -> esporta_firmware.py.
 */
#ifndef CONFIG_CONTROLLO_H
#define CONFIG_CONTROLLO_H

#define CFG_SWING_UP              1
#define CFG_ANGOLO_INIZIALE_RAD   (-3.141592653589793f)

/* Periodo della rete ([episodio].periodo_controllo_s): letture, stimatore, inferenza. */
#define CFG_PERIODO_CONTROLLO_MS  20U

/* Motore: stessi valori di [stepper] (stepper.py). */
#define CFG_MICROSTEP             32
#define CFG_FREQ_MIN_HZ           3200.0f
#define CFG_FREQ_MAX_HZ           32000.0f
#define CFG_RAMPA_HZ_S            1600000.0f
#define CFG_ZONA_MORTA            0.0f
#define CFG_METRI_PER_STEP        1.5e-05f
/* Inversione: periodi di rampa (2 ms) con frequenza ferma a zero (stepper.py). */
#define CFG_TICK_FERMO_INVERSIONE 1U

/* Scale dei sei ingressi della rete (come l ambiente Python). */
#define CFG_POSIZIONE_SCALA_M     0.462f
#define CFG_VELOCITA_SCALA_M_S    0.48000000000000004f
#define CFG_OMEGA_SCALA_RAD_S     10.0f

/* Arresto del controllo a |x| >= limite (dal centro): lo stesso dell addestramento. */
#define CFG_LIMITE_CORSA_M        0.4f

/* Encoder e controlli di sicurezza ([firmware]). */
#define CFG_ENCODER_CPR           2400
#define CFG_RICALIBRA_THETA       1
#define CFG_ANGOLO_RICALIBRAZIONE_RAD 0.3490658503988659f
#define CFG_TEMPO_QUIETE_MS       1000U
#define CFG_ANGOLO_MAX_AVVIO_RAD  0.17453292519943295f
#define CFG_ANGOLO_MAX_CONTROLLO_RAD 0.7853981633974483f
#define CFG_SALTO_ENCODER_MAX     191U  /* conteggi in 20 ms = 25 rad/s */

/* Stimatore (stima.c), [stimatore]. */
#define STIMA_K_G      17.3295f
#define STIMA_K_A      1.76651f
#define STIMA_K_D      0.403267f
#define STIMA_L_THETA  0.259944f
#define STIMA_L_OMEGA  1.187222f
#define STIMA_L_X      0.05f
/* Offset dell encoder stimato (0 = stimatore a due stati). */
#define STIMA_L_B      (-0.044103f)
#define STIMA_B_MAX_RAD 0.261799f

#endif
