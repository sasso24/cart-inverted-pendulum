/* GENERATO da cart_pendolo_velocita_rl/esporta_firmware.py: NON modificare a mano. */
#ifndef RETI_CANDIDATE_H
#define RETI_CANDIDATE_H

#include "ai_platform.h"

#define RETI_CANDIDATE_N 2U

typedef struct { float l_theta, l_omega, l_b, l_x; } GuadagniStima;

/* Pesi (formato X-CUBE-AI di network_data_params.c), guadagni dello stimatore
 * e nome di ogni rete. */
extern const ai_handle reti_candidate_pesi[RETI_CANDIDATE_N];
extern const GuadagniStima reti_candidate_stima[RETI_CANDIDATE_N];
extern const char *const reti_candidate_nomi[RETI_CANDIDATE_N];

#endif
