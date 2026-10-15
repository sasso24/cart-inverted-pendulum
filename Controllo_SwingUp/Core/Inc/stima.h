/* Stimatore dello stato del pendolo: porting 1:1 di
 * cart_pendolo_velocita_rl/stima.py (costanti da config_controllo.h).
 *
 * Predizione (500 Hz, ogni passo della rampa, velocita' applicata v):
 *   a      = (v - v_precedente) / dt
 *   alfa   = K_G*sin(theta) - K_A*cos(theta)*a - K_D*omega
 *   omega += dt*alfa ; theta += dt*omega ; x += dt*v
 * Correzione (50 Hz, posizione e angolo letti):
 *   r = wrap(theta_mis - theta) ; theta += L_THETA*r ; omega += L_OMEGA*r
 *   x += L_X*(x_mis - x)
 * La rete e' stata addestrata con questi valori stimati come ingressi.
 */
#ifndef STIMA_H
#define STIMA_H

/* Costanti STIMA_K_G, STIMA_K_A, STIMA_K_D, STIMA_L_THETA, STIMA_L_OMEGA,
 * STIMA_L_X: in config_controllo.h, generato da esporta_firmware.py con i
 * valori di [stimatore] salvati insieme alla rete. */
#include "config_controllo.h"

typedef struct
{
    float x;                    /* m, rispetto al centro della corsa */
    float theta;                /* rad, 0 in alto, in (-pi, pi]      */
    float omega;                /* rad/s                             */
    float velocita;             /* m/s, ultima velocita' applicata   */
    float velocita_precedente;  /* m/s, per l'accelerazione          */
} Stima;

/* Prima lettura: pendolo considerato fermo. */
void Stima_Reset(Stima *s, float x, float theta, float velocita);
/* Un passo della rampa (dt = 0,002 s). */
void Stima_Predici(Stima *s, float velocita_applicata, float dt);
/* Una lettura dei sensori (ogni 20 ms). */
void Stima_Correggi(Stima *s, float x_misurata, float theta_misurato);
/* Angolo riportato in (-pi, pi]. */
float Stima_Wrap(float angolo);

#endif
