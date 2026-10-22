/* Stimatore dello stato del pendolo: porting 1:1 di
 * cart_pendolo_velocita_rl/stima.py (costanti da config_controllo.h).
 *
 * Predizione (500 Hz, ogni passo della rampa, velocita' applicata v):
 *   a      = (v - v_precedente) / dt
 *   alfa   = K_G*sin(theta) - K_A*cos(theta)*a - K_D*omega
 *   omega += dt*alfa ; theta += dt*omega ; x += dt*v
 * Correzione (una volta per periodo della rete, posizione e angolo letti):
 *   r = wrap(theta_mis - theta - offset)
 *   offset += L_B*max(cos(theta), 0)*r   (limitato a +/-B_MAX_RAD; L_B = 0: niente offset)
 *   theta += L_THETA*r ; omega += L_OMEGA*r ; x += L_X*(x_mis - x)
 * offset = errore dell'encoder (conteggi persi, disturbi) stimato dalla
 * dinamica: theta e' l'angolo vero stimato, quello che riceve la rete.
 * La rete e' stata addestrata con questi valori stimati come ingressi.
 */
#ifndef STIMA_H
#define STIMA_H

/* Costanti STIMA_K_G, STIMA_K_A, STIMA_K_D, STIMA_L_THETA, STIMA_L_OMEGA,
 * STIMA_L_X: in config_controllo.h, generato da esporta_firmware.py con i
 * valori di [stimatore] salvati insieme alla rete. */
#include "config_controllo.h"

/* Header generati prima del 16/10: stimatore a due stati. */
#ifndef STIMA_L_B
#define STIMA_L_B      0.0f
#endif
#ifndef STIMA_B_MAX_RAD
#define STIMA_B_MAX_RAD 0.2617994f
#endif

typedef struct
{
    float x;                    /* m, rispetto al centro della corsa */
    float theta;                /* rad, 0 in alto, in (-pi, pi]      */
    float omega;                /* rad/s                             */
    float velocita;             /* m/s, ultima velocita' applicata   */
    float velocita_precedente;  /* m/s, per l'accelerazione          */
    float offset;               /* rad, errore stimato dell'encoder  */
} Stima;

/* Guadagni di correzione in uso: all'avvio quelli di config_controllo.h,
 * poi quelli della rete scelta (reti_candidate.c, main.c). */
extern float stima_l_theta, stima_l_omega, stima_l_b, stima_l_x;

/* Prima lettura: pendolo considerato fermo. */
void Stima_Reset(Stima *s, float x, float theta, float velocita);
/* Un passo della rampa (dt = 0,002 s). */
void Stima_Predici(Stima *s, float velocita_applicata, float dt);
/* Una lettura dei sensori (una per periodo della rete). */
void Stima_Correggi(Stima *s, float x_misurata, float theta_misurato);
/* Angolo riportato in (-pi, pi]. */
float Stima_Wrap(float angolo);

#endif
