/* GENERATO da cart_pendolo_velocita_rl/esporta_firmware.py: NON modificare a mano.
 * Reti selezionabili con rete_scelta (main.c), tutte con config_controllo.h:
 *   0: modelli_asta20_perno_storia1s_20s/migliore.zip
 */
#include "reti_candidate.h"
#include "network_data.h"

const ai_handle reti_candidate_pesi[RETI_CANDIDATE_N] = {
  AI_HANDLE_PTR(s_network_weights_array_u64),
};

const GuadagniStima reti_candidate_stima[RETI_CANDIDATE_N] = {
  { 0.256295f, 1.187233f, (-0.040454f), 0.05f },
};

const char *const reti_candidate_nomi[RETI_CANDIDATE_N] = {
  "modelli_asta20_perno_storia1s_20s/migliore.zip",
};
