/* Porting di cart_pendolo_velocita_rl/stima.py: stesso ordine delle operazioni.
 * Nessuna dipendenza da HAL: compilabile e testabile anche su host.
 */
#include "stima.h"
#include <math.h>

#define STIMA_PI     3.14159265358979323846f
#define STIMA_2PI    6.28318530717958647692f

float Stima_Wrap(float angolo)
{
    /* Come math.remainder(angolo, 2*pi) ma con +pi incluso: (-pi, pi]. */
    angolo = remainderf(angolo, STIMA_2PI);
    if (angolo <= -STIMA_PI)
        angolo += STIMA_2PI;
    return angolo;
}

void Stima_Reset(Stima *s, float x, float theta, float velocita)
{
    s->x = x;
    s->theta = theta;
    s->omega = 0.0f;
    s->velocita = velocita;
    s->velocita_precedente = velocita;
}

void Stima_Predici(Stima *s, float velocita_applicata, float dt)
{
    float accelerazione = (velocita_applicata - s->velocita_precedente) / dt;
    s->velocita_precedente = velocita_applicata;
    float alfa = STIMA_K_G * sinf(s->theta)
               - STIMA_K_A * cosf(s->theta) * accelerazione
               - STIMA_K_D * s->omega;
    s->omega += dt * alfa;
    s->theta += dt * s->omega;
    s->x += dt * velocita_applicata;
    s->velocita = velocita_applicata;
}

void Stima_Correggi(Stima *s, float x_misurata, float theta_misurato)
{
    float r = Stima_Wrap(theta_misurato - s->theta);
    s->theta = Stima_Wrap(s->theta + STIMA_L_THETA * r);
    s->omega += STIMA_L_OMEGA * r;
    s->x += STIMA_L_X * (x_misurata - s->x);
}
