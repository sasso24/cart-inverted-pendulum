"""Esegue su host il codice USER CODE reale con HAL e rete simulate.

python3 Controllo_SwingUp_Asta20_Storia/tests/test_controllo.py
Usa Core/Inc/config_controllo.h cosi com'e' (swing-up atteso: CFG_SWING_UP 1).
Non verifica il runtime ARM, la dinamica meccanica o i segnali elettrici.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MOCK = r'''
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t CR1, PSC, EGR, ARR, CNT, CCR; } Timer;
typedef struct { Timer *Instance; uint32_t Channel; } TIM_HandleTypeDef;
typedef int UART_HandleTypeDef;
static Timer timer2, timer3;
#define TIM3 (&timer3)
#define TIM_CHANNEL_1 1
#define TIM_FLAG_UPDATE 1
#define TIM_FLAG_CC1 2
#define TIM_HAL_UNUSED 0
#define HAL_TIM_ACTIVE_CHANNEL_1 1
#define TIM_CR1_UDIS 1
#define TIM_EGR_UG 1
#define B1_GPIO_Port 0
#define FC_sx_GPIO_Port 0
#define Fc_dx_GPIO_Port 0
#define DIR_GPIO_Port 0
#define LD2_GPIO_Port 0
#define B1_Pin 1
#define FC_sx_Pin 2
#define Fc_dx_Pin 3
#define DIR_Pin 4
#define LD2_Pin 5
static GPIO_PinState pins[6];
static uint32_t tick, primask, starts;
static int fail_pwm;
static void (*delay_hook)(void);
static struct { uint32_t CFGR; } rcc;
#define RCC (&rcc)
#define RCC_CFGR_PPRE1 1
#define __HAL_TIM_GET_COUNTER(h) ((h)->Instance->CNT)
#define __HAL_TIM_SET_COUNTER(h,v) ((h)->Instance->CNT=(v))
#define __HAL_TIM_SET_AUTORELOAD(h,v) ((h)->Instance->ARR=(v))
#define __HAL_TIM_SET_COMPARE(h,c,v) ((h)->Instance->CCR=(v))
#define __HAL_TIM_CLEAR_FLAG(h,f) ((void)0)
#define SET_BIT(r,b) ((r)|=(b))
#define CLEAR_BIT(r,b) ((r)&=~(b))
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask=1; }
static void __set_PRIMASK(uint32_t v) { primask=v; }
static uint32_t HAL_GetTick(void) { return tick; }
static void HAL_Delay(uint32_t v) { tick+=v; if (delay_hook) delay_hook(); }
static uint32_t HAL_RCC_GetPCLK1Freq(void) { return 90000000; }
static GPIO_PinState HAL_GPIO_ReadPin(int port, int pin) { return pins[pin]; }
static void HAL_GPIO_WritePin(int port, int pin, GPIO_PinState v) { pins[pin]=v; }
static HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *h, int c) { return HAL_OK; }
static HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *h, int c) {
    ++starts; return fail_pwm ? HAL_ERROR : HAL_OK;
}
typedef void *ai_handle;
typedef unsigned char ai_u8;
typedef float ai_float;
typedef int ai_i32;
typedef struct { int type, code; } ai_error;
typedef struct { ai_handle data; } ai_buffer;
#define AI_HANDLE_NULL NULL
#define AI_HANDLE_PTR(x) ((void *)(x))
#define AI_ALIGNED(x) __attribute__((aligned(x)))
#define AI_ERROR_NONE 0
#define AI_NETWORK_IN_1_SIZE CFG_STORIA_INGRESSI   /* da config_controllo.h */
#define AI_NETWORK_OUT_1_SIZE 1
#define AI_NETWORK_DATA_ACTIVATIONS_SIZE 1024
static uint32_t P_mock;   /* periodo della rete, impostato da reset() */
static float input_values[512], output_value;   /* 6 + 7 per ogni ritardo della storia */
static ai_buffer input_buffer = {input_values}, output_buffer = {&output_value};
static int batches=1, fail_init, during_run;
void HAL_GPIO_EXTI_Callback(uint16_t);
void Controllo_MotorTick(void);
/* Reti selezionabili (reti_candidate.c): qui due array finti, si controlla il puntatore. */
#define RETI_CANDIDATE_N 2U
static const char pesi_finti[2][4];
static const ai_handle reti_candidate_pesi[RETI_CANDIDATE_N] = {
    (ai_handle)pesi_finti[0], (ai_handle)pesi_finti[1] };
typedef struct { float l_theta, l_omega, l_b, l_x; } GuadagniStima;
static GuadagniStima reti_candidate_stima[RETI_CANDIDATE_N];   /* riempita da reset() */
static const void *pesi_usati; static int distrutte;
static ai_error ai_network_create_and_init(ai_handle *h, const ai_handle *a, const ai_handle *w) {
    pesi_usati = w ? w[0] : NULL;
    *h=input_values; return (ai_error){fail_init, 0};
}
static ai_handle ai_network_destroy(ai_handle h) { ++distrutte; return AI_HANDLE_NULL; }
static ai_buffer *ai_network_inputs_get(ai_handle h, void *n) { return &input_buffer; }
static ai_buffer *ai_network_outputs_get(ai_handle h, void *n) { return &output_buffer; }
static ai_error ai_network_get_error(ai_handle h) { return (ai_error){2, 42}; }
static ai_i32 ai_network_run(ai_handle h, const ai_buffer *i, ai_buffer *o) {
    if (during_run==1) HAL_GPIO_EXTI_Callback(B1_Pin);
    if (during_run==2) tick+=P_mock;
    if (during_run==3) { tick+=2*P_mock; Controllo_MotorTick(); }
    return batches;
}
'''
TESTS = r'''
#define P CONTROL_PERIOD_MS   /* periodo della rete da config_controllo.h */
static void reset(void) {
    P_mock=P;
    reti_candidate_stima[0] = (GuadagniStima){ STIMA_L_THETA, STIMA_L_OMEGA, STIMA_L_B, STIMA_L_X };
    reti_candidate_stima[1] = (GuadagniStima){ 0.2f, 1.0f, 0.0f, 0.05f };
    Controllo_Disabilita();
    delay_hook=NULL; theta_inizializzato=1;
    memset(pins,0,sizeof pins);
    tick=0; starts=0; primask=0; fail_init=0; fail_pwm=0; during_run=0; batches=1;
    htim2.Instance=&timer2; htim3.Instance=&timer3;
    htim3.Channel=HAL_TIM_ACTIVE_CHANNEL_1;
    timer2.CNT=0; timer3.PSC=83;
    stepper_counter_clock=90000000.0f/84.0f;
    conteggio_precedente=0; posizione=HALF_CPR; theta_rad=PI_F;
    ultimo_theta_rad=PI_F; ultimo_campione_ms=0;
    posizione_carrello_steps=30800; ultimo_carrello_steps=30800;
    posizione_carrello_min_steps=0; posizione_carrello_max_steps=61600;
    corsa_totale_steps=61600; Calcola_LimitiControllo();
    encoder_quiete_ms=CFG_TEMPO_QUIETE_MS; encoder_salti=0; theta_ricalibrazioni=0;
    riferimento_carrello_valido=1; carrello_inizializzato=1;
    pulsante_inizializzato=1; pulsante_pronto=1; pulsante_rilascio_ms=0;
    richiesta_avvio=0; rete_inferenze=0; controllo_errore=0;
    stima_valida=0; ultimo_stima_ms=0;
    Stepper_SetDirection(DIR_BACKWARD);
    output_value=0.5f;
    assert(Rete_Init());
}
static void start(void) {
    richiesta_avvio=1; Controllo_AccettaAvvio(); assert(controllo_abilitato);
}
static void near(float a, float b) {
    if (fabsf(a-b)>=0.0002f+fabsf(b)*1e-6f) fprintf(stderr,"actual=%g expected=%g\n",a,b);
    assert(fabsf(a-b)<0.0002f+fabsf(b)*1e-6f);
}
static void pulse(void) { HAL_TIM_PWM_PulseFinishedCallback(&htim3); }
static void moving_encoder(void) { ++timer2.CNT; }
static void settling_encoder(void) {
    if (tick < 1500U) timer2.CNT = tick % 2U;
}
int main(void) {
    // Il riferimento viene acquisito sul conteggio reale prima dell'homing.
    reset(); timer2.CNT=12345; theta_rad=0.8f;
    assert(Inizializza_Theta()==HAL_OK);
    assert(tick==THETA_STABILE_MS && theta_inizializzato && starts==0);
    assert(conteggio_precedente==12345 && timer2.CNT==12345);
    near(theta_rad,PI_F); near(theta_deg,180);
    // Moto durante l'homing: nessun nuovo azzeramento dell'angolo.
    timer2.CNT+=100; Leggi_theta(); near(theta_rad,Stima_Wrap(PI_F+ENCODER_SIGN*100*2*PI_F/2400));
    // Una pausa all'estremo di un'oscillazione non basta per calibrare.
    reset(); delay_hook=settling_encoder;
    assert(Inizializza_Theta()==HAL_OK);
    assert(tick>=2490U && tick<2510U && starts==0);
    // Pendolo ancora mobile: scadenza, riferimento invalido, motore fermo.
    reset(); delay_hook=moving_encoder;
    assert(Inizializza_Theta()==HAL_TIMEOUT);
    assert(!theta_inizializzato && !stepper_running && starts==0);
    richiesta_avvio=1; Controllo_AccettaAvvio(); assert(!controllo_abilitato);
    // Stabilita' anche oltre il wrap del tick; primo delta encoder oltre 32 bit.
    reset(); tick=UINT32_MAX-500; timer2.CNT=UINT32_MAX;
    assert(Inizializza_Theta()==HAL_OK && starts==0);
    timer2.CNT=0; Leggi_theta(); near(theta_rad,Stima_Wrap(PI_F+ENCODER_SIGN*2*PI_F/2400));

    reset(); start(); tick=P; Rete_Process();
    near(input_values[0],0); near(input_values[1],0); near(input_values[2],-1);
    near(input_values[3],0); near(input_values[4],0); near(input_values[5],0);
    near(frequenza_richiesta_hz,17600); assert(rete_inferenze==1);
    tick=P+2; Controllo_MotorTick(); assert(stepper_running);
    near(frequenza_rampa_hz,3200); assert(frequenza_applicata_hz>0);
    pulse(); assert(posizione_carrello_steps==30801);
    // Stop B1 immediato, nessun riavvio nei tick successivi; antirimbalzo.
    HAL_GPIO_EXTI_Callback(B1_Pin); assert(!controllo_abilitato && !stepper_running);
    near(frequenza_richiesta_hz,0); near(frequenza_applicata_hz,0);
    HAL_GPIO_EXTI_Callback(B1_Pin); assert(!richiesta_avvio);
    tick=P+4; Controllo_MotorTick(); assert(!stepper_running);
    pins[B1_Pin]=GPIO_PIN_SET;
    for(int i=0;i<51;i++) Controllo_ButtonTick();
    pins[B1_Pin]=GPIO_PIN_RESET; HAL_GPIO_EXTI_Callback(B1_Pin);
    assert(richiesta_avvio); Controllo_AccettaAvvio(); assert(controllo_abilitato);

    // I sei ingressi, unita' e attraversamento +/-pi senza impulso su omega.
    // All'avvio lo stimatore riparte dalla lettura (x=0, theta=pi, fermo);
    // la prima lettura del controllo e' gia' una correzione.
    reset(); start(); near(stima.x,0); near(stima.omega,0);
    tick=P; timer2.CNT=(uint32_t)(2*ENCODER_SIGN);
    posizione_carrello_steps+=640; frequenza_applicata_hz=-16000;
    Rete_Process();
    near(input_values[0],STIMA_L_X*0.0096f/0.462f); near(input_values[3],0);
    near(input_values[5],-0.5f);
    near(theta_rad,-PI_F+2*2*PI_F/2400);
    near(stima.theta,Stima_Wrap(PI_F+STIMA_L_THETA*2*2*PI_F/2400));
    near(input_values[4],STIMA_L_OMEGA*2*2*PI_F/2400/10);
    // Lo stimatore integra la velocita' STEP programmata ogni 2 ms in SysTick;
    // le letture rumorose dell'angolo vengono attenuate (L_THETA=0.2).
    reset(); start(); tick=P; Rete_Process();
    stepper_running=1; frequenza_applicata_hz=3200;
    for (int i=0;i<10;i++){ tick+=2; Controllo_StimaTick(); }
    near(stima.x,10*0.002f*3200*METRI_PER_STEP); near(stima.velocita,0.048f);
    timer2.CNT+=(uint32_t)(24*ENCODER_SIGN); tick=2*P; Rete_Process();
    near(input_values[3],0.1f);
    near(input_values[0],stima.x/0.462f);
    // 24 conteggi (3,6 gradi) in un periodo: la derivata darebbe omega=3,1 rad/s (20 ms),
    // lo stimatore ne prende solo una frazione.
    assert(input_values[4]>0 && input_values[4]*10<0.5f*(24*2*PI_F/2400)/(P*0.001f));
    // A controllo fermo: v=0 e x agganciata al conteggio STEP.
    Controllo_Disabilita(); float xprima=stima.x;
    for (int i=0;i<5;i++){ tick+=2; Controllo_StimaTick(); }
    near(stima.x,xprima); near(stima.velocita,0);
    tick=3*P; Rete_Process(); near(stima.x,0);

    // Uscita zero e saturazione, senza seconda tanh.
    reset(); start(); tick=P; Rete_Process();
    output_value=0; tick=2*P; Rete_Process(); near(frequenza_richiesta_hz,0);
    output_value=-2; tick=3*P; Rete_Process(); near(frequenza_richiesta_hz,-32000);

    // Decelerazione e inversione solo dopo un impulso completo e setup DIR.
    reset(); start(); frequenza_richiesta_hz=6400;
    tick=2; Controllo_MotorTick(); tick=4; Controllo_MotorTick();
    near(frequenza_rampa_hz,6400);
    frequenza_richiesta_hz=-6400;
    tick=6; Controllo_MotorTick(); near(frequenza_rampa_hz,3200);
    tick=8; Controllo_MotorTick(); assert(stop_al_fronte_step && stepper_running);
    // L'ISR ferma il timer al fronte e imposta subito il nuovo DIR: la rampa
    // riparte da zero al periodo successivo, come stepper.py (+3200, 0, -3200).
    pulse(); assert(!stepper_running && stepper_direction==DIR_FORWARD);
    near(frequenza_applicata_hz,0); near(frequenza_rampa_hz,0);
    tick=10; Controllo_MotorTick(); assert(stepper_running && frequenza_applicata_hz<0);
    near(frequenza_rampa_hz,-3200);
    int32_t before=posizione_carrello_steps; pulse(); assert(posizione_carrello_steps==before-1);
    frequenza_richiesta_hz=0;
    tick=12; Controllo_MotorTick(); tick=14; Controllo_MotorTick(); pulse();
    assert(!stepper_running); near(frequenza_applicata_hz,0);
    // Se il fronte tarda, la rampa resta a zero (nessun salto alla ripartenza).
    reset(); start(); frequenza_richiesta_hz=6400;
    tick=2; Controllo_MotorTick(); tick=4; Controllo_MotorTick();
    frequenza_richiesta_hz=-6400;
    tick=6; Controllo_MotorTick(); tick=8; Controllo_MotorTick(); assert(stop_al_fronte_step);
    tick=10; Controllo_MotorTick(); near(frequenza_rampa_hz,0); assert(stepper_running);
    pulse(); assert(!stepper_running && stepper_direction==DIR_FORWARD);
    tick=12; Controllo_MotorTick(); near(frequenza_rampa_hz,-3200); assert(stepper_running);
#if CFG_TICK_FERMO_INVERSIONE == 1
    // Attraversamento dello zero (+1600 -> -1600): un periodo fermo a zero.
    reset(); start(); frequenza_richiesta_hz=4800;
    tick=2; Controllo_MotorTick(); near(frequenza_rampa_hz,3200);
    frequenza_richiesta_hz=1600; tick=4; Controllo_MotorTick(); near(frequenza_rampa_hz,1600);
    frequenza_richiesta_hz=-6400; tick=6; Controllo_MotorTick();
    near(frequenza_rampa_hz,0); assert(stop_al_fronte_step);
    pulse(); assert(!stepper_running && stepper_direction==DIR_FORWARD);
    tick=8; Controllo_MotorTick(); near(frequenza_rampa_hz,-3200); assert(stepper_running);
#endif

    // B1 durante l'inferenza non permette di pubblicare il vecchio comando.
    reset(); start(); during_run=1; tick=P; Rete_Process();
    assert(!controllo_abilitato); near(frequenza_richiesta_hz,0); assert(rete_inferenze==0);
    // Errori rete, valori non finiti e deadline fermano il controllo.
    reset(); start(); batches=0; tick=P; Rete_Process();
    assert(controllo_errore==2 && !rete_pronta && !controllo_abilitato);
    reset(); start(); output_value=NAN; tick=P; Rete_Process();
    assert(controllo_errore==3 && !controllo_abilitato);
    reset(); start(); during_run=2; tick=P; Rete_Process();
    assert(controllo_errore==5 && !controllo_abilitato);
    reset(); start(); during_run=3; tick=P; Rete_Process();
    assert(controllo_errore==5 && !controllo_abilitato);
    reset(); start(); tick=2*P; Controllo_MotorTick();
    assert(controllo_errore==5 && !controllo_abilitato);
    // Finecorsa dal tick e dall'ISR STEP: fault latched, nessuna ripartenza.
    reset(); start(); pins[FC_sx_Pin]=GPIO_PIN_SET; tick=1; Controllo_MotorTick();
    assert(controllo_errore==4 && !controllo_abilitato);
    richiesta_avvio=1; Controllo_AccettaAvvio(); assert(!controllo_abilitato);
    reset(); start(); frequenza_richiesta_hz=3200; tick=2; Controllo_MotorTick();
    pins[Fc_dx_Pin]=GPIO_PIN_SET; pulse();
    assert(controllo_errore==4 && !controllo_abilitato && !stepper_running);
    reset(); start(); posizione_carrello_steps=posizione_carrello_max_steps;
    tick=1; Controllo_MotorTick(); assert(controllo_errore==4);
    // Limite del controllo = centro +/- CFG_LIMITE_CORSA_M, prima dei finecorsa.
    assert(limite_controllo_min_steps==(int32_t)ceilf(30800-CFG_LIMITE_CORSA_M/METRI_PER_STEP));
    assert(limite_controllo_max_steps==(int32_t)floorf(30800+CFG_LIMITE_CORSA_M/METRI_PER_STEP));
    assert(limite_controllo_max_steps<posizione_carrello_max_steps);
    reset(); start(); posizione_carrello_steps=limite_controllo_max_steps-1;
    tick=1; Controllo_MotorTick(); assert(controllo_abilitato);
    frequenza_richiesta_hz=3200; tick=2; Controllo_MotorTick(); assert(stepper_running);
    pulse(); assert(controllo_errore==4 && !controllo_abilitato && !stepper_running);
    reset(); posizione_carrello_steps=limite_controllo_min_steps;
    richiesta_avvio=1; Controllo_AccettaAvvio(); assert(!controllo_abilitato);
    reset(); start(); fail_pwm=1; frequenza_richiesta_hz=3200; tick=2; Controllo_MotorTick();
    assert(controllo_errore==6 && !controllo_abilitato);
    reset(); fail_init=1; assert(!Rete_Init());
    // Tick wraparound.
    reset(); tick=UINT32_MAX-10; start(); tick=9; ultimo_campione_ms=UINT32_MAX-10;
    Rete_Process(); assert(rete_inferenze==1 && controllo_abilitato);
#if CFG_SWING_UP && CFG_RICALIBRA_THETA
    // Swing-up: B1 accettato solo con pendolo fermo; l'angolo torna a +/-180
    // gradi, cancellando l'errore accumulato dall'encoder.
    reset(); posizione=HALF_CPR-37; theta_rad=(float)posizione*2*PI_F/ENCODER_CPR;
    encoder_quiete_ms=CFG_TEMPO_QUIETE_MS-20;
    richiesta_avvio=1; Controllo_AccettaAvvio();
    assert(!controllo_abilitato && controllo_errore==11 && theta_ricalibrazioni==0);
    encoder_quiete_ms=CFG_TEMPO_QUIETE_MS; timer2.CNT=777;
    richiesta_avvio=1; Controllo_AccettaAvvio();
    assert(controllo_abilitato && controllo_errore==0 && theta_ricalibrazioni==1);
    near(fabsf(theta_rad),PI_F); assert(conteggio_precedente==777);
    near(fabsf(stima.theta),PI_F); near(stima.omega,0);
    assert(avvio_modo==1);
    // Pendolo portato su a mano (anche se trema): nessuna ricalibrazione,
    // partenza diretta con l'equilibrio dall'angolo letto.
    reset(); encoder_quiete_ms=0; posizione=5; theta_rad=5*2*PI_F/ENCODER_CPR;
    richiesta_avvio=1; Controllo_AccettaAvvio();
    assert(controllo_abilitato && controllo_errore==0 && avvio_modo==2);
    assert(theta_ricalibrazioni==0 && posizione==5);
    near(stima.theta,5*2*PI_F/ENCODER_CPR); near(stima.omega,0);
    tick=P; Rete_Process(); near(input_values[2],cosf(5*2*PI_F/ENCODER_CPR));
    // Lo stesso anche con l'encoder fermo: in alto l'angolo non viene mai riazzerato.
    reset(); posizione=-60; theta_rad=-60*2*PI_F/ENCODER_CPR;
    richiesta_avvio=1; Controllo_AccettaAvvio();
    assert(controllo_abilitato && avvio_modo==2 && theta_ricalibrazioni==0 && posizione==-60);
    // Soglie: in alto entro CFG_ANGOLO_MAX_AVVIO_RAD, in basso entro
    // CFG_ANGOLO_RICALIBRAZIONE_RAD; in mezzo avvio rifiutato (errore 11).
    {
        int32_t alto_ok = (int32_t)floorf(CFG_ANGOLO_MAX_AVVIO_RAD*ENCODER_CPR/(2*PI_F));
        int32_t basso_ok = (int32_t)floorf(CFG_ANGOLO_RICALIBRAZIONE_RAD*ENCODER_CPR/(2*PI_F));
        int32_t prove[][2] = { { alto_ok, 1 }, { -alto_ok, 1 }, { alto_ok+2, 0 },
                               { HALF_CPR-basso_ok, 1 }, { -HALF_CPR+basso_ok, 1 },
                               { HALF_CPR-basso_ok-2, 0 }, { 600, 0 }, { -600, 0 } };
        for (unsigned i=0;i<sizeof prove/sizeof prove[0];i++) {
            reset(); posizione=prove[i][0]; theta_rad=(float)posizione*2*PI_F/ENCODER_CPR;
            richiesta_avvio=1; Controllo_AccettaAvvio();
            if (prove[i][1]) assert(controllo_abilitato && controllo_errore==0);
            else assert(!controllo_abilitato && controllo_errore==11 && avvio_modo==0);
        }
    }
    // La quiete si accumula a letture con al piu' un conteggio e si azzera col moto.
    reset(); encoder_quiete_ms=0;
    for (int i=1;i<=(int)(1000/P);i++){ tick=P*i; timer2.CNT=(uint32_t)(i%2); Rete_Process(); }
    assert(encoder_quiete_ms==1000);
    timer2.CNT+=5; tick=1000+P; Rete_Process(); assert(encoder_quiete_ms==0);
#endif
    // Salto impossibile dell'encoder (disturbo): arresto con errore 9.
    reset(); start(); tick=P; timer2.CNT=CFG_SALTO_ENCODER_MAX+1; Rete_Process();
    assert(controllo_errore==9 && !controllo_abilitato && encoder_salti==1);
    reset(); start(); tick=P; timer2.CNT=CFG_SALTO_ENCODER_MAX; Rete_Process();
    assert(controllo_abilitato && rete_inferenze==1);
    // Periodo senza deriva: il campione successivo non slitta.
    reset(); start(); tick=P+1; Rete_Process(); assert(ultimo_campione_ms==P);
    tick=2*P; Rete_Process(); assert(rete_inferenze==2 && ultimo_campione_ms==2*P);
#if CFG_TICK_FERMO_INVERSIONE > 0
    // Rampa del firmware = ControlloStepper.avanza (stepper.py) su sequenze casuali
    // con molte inversioni (attese calcolate in Python, vedi sotto). Con 0 (reti
    // fino al 14/10) la simulazione attraversa lo zero senza fermarsi, il firmware no.
    reset(); start();
    for (unsigned i=0;i<sizeof RAMPA_RICHIESTE/sizeof RAMPA_RICHIESTE[0];i++) {
        frequenza_richiesta_hz=RAMPA_RICHIESTE[i]; ultimo_comando_ms=tick;
        tick+=2; Controllo_MotorTick();
        if (stop_al_fronte_step) pulse();   // il fronte arriva prima del periodo successivo
        assert(controllo_abilitato);
        if (fabsf(frequenza_rampa_hz-RAMPA_ATTESE[i])>0.01f) fprintf(stderr,"i=%u req=%g prev=%g\n",i,RAMPA_RICHIESTE[i],i?RAMPA_ATTESE[i-1]:0.0f);
        near(frequenza_rampa_hz,RAMPA_ATTESE[i]);
        if (RAMPA_ATTESE[i]!=0.0f) assert(stepper_running
            && (RAMPA_ATTESE[i]>0) == (stepper_direction==DIR_BACKWARD));
    }
    puts("OK: rampa e inversioni identiche a stepper.py");
#endif
    // Cambio di rete dal debugger: solo a controllo fermo, indice valido.
    reset(); assert(pesi_usati==pesi_finti[0] && rete_attiva==0);
    rete_scelta=1; start(); Rete_CambiaSeRichiesto(); assert(rete_attiva==0);  // in controllo: no
    Controllo_Disabilita(); distrutte=0; Rete_CambiaSeRichiesto();
    assert(rete_attiva==1 && pesi_usati==pesi_finti[1] && distrutte==1 && rete_pronta);
    near(stima_l_theta,0.2f); near(stima_l_omega,1.0f); near(stima_l_b,0.0f); near(stima_l_x,0.05f);
    rete_scelta=7; Rete_CambiaSeRichiesto(); assert(rete_attiva==1);            // fuori intervallo
    rete_scelta=0; fail_init=1; Rete_CambiaSeRichiesto();
    assert(rete_attiva==0 && !rete_pronta && controllo_errore==1);
    fail_init=0; rete_scelta=0; rete_attiva=0; assert(Rete_Init());
    near(stima_l_theta,STIMA_L_THETA); near(stima_l_b,STIMA_L_B);
    puts("OK: cambio di rete (rete_scelta) solo a controllo fermo");
#if CFG_STORIA_N > 0
    // Storia (storia.py): ingressi e azione di k periodi fa, all'avvio i primi ingressi e u=0.
    {
        static const uint32_t rit[CFG_STORIA_N] = CFG_STORIA_RITARDI;
        static float oss[80][6], az[80];
        for (int giro=0; giro<2; giro++) {
            if (giro==0) { reset(); start(); }
            else { Controllo_Disabilita(); encoder_quiete_ms=CFG_TEMPO_QUIETE_MS; start(); }  // riavvio: storia dai primi ingressi
            uint32_t t0=tick;
            for (int i=1;i<=70;i++) {
                output_value=1.5f*sinf(0.7f*(float)(i+giro));
                timer2.CNT+=(uint32_t)(i%3); tick=t0+P*(uint32_t)i; Rete_Process();
                assert(controllo_abilitato && rete_inferenze>0);
                for (int c=0;c<6;c++) { oss[i][c]=rete_osservazione[c]; assert(input_values[c]==oss[i][c]); }
                az[i]=fmaxf(-1.0f,fminf(output_value,1.0f));
                for (uint32_t j=0;j<CFG_STORIA_N;j++) {
                    int s=i-(int)rit[j];
                    for (int c=0;c<6;c++) assert(input_values[6+7*j+c]==oss[s>=1?s:1][c]);
                    assert(input_values[6+7*j+6]==(s>=1?az[s]:0.0f));
                }
            }
        }
    }
    puts("OK: storia degli ingressi identica a storia.py (ritardi, azioni, riavvio)");
#endif
    puts("OK: calibrazione theta, ricalibrazione all'avvio solo da appeso, partenza dall'alto, salti encoder, limiti di corsa, "
         "ingressi, azioni, rampa, inversione, B1, fault, deadline e wraparound");
}
'''

def rampa_attesa(config_h):
    """Richieste casuali (multipli di 100 Hz, esatti in float32) e frequenze di rampa
    attese da stepper.py con i parametri dell'header."""
    import random
    import re
    import sys
    from dataclasses import replace
    sys.path.insert(0, str(ROOT.parent / 'cart_pendolo_velocita_rl'))
    from stepper import ControlloStepper, ParametriStepper

    def valore(nome, default=None):
        trovato = re.search(rf'#define {nome}\s+\(?(-?[0-9.e+-]+)f?U?', config_h)
        if trovato is None and default is not None:
            return default    # header precedente: stesso default del firmware
        return float(trovato[1])
    p = replace(ParametriStepper.carica(), frequenza_min_hz=valore('CFG_FREQ_MIN_HZ'),
                frequenza_max_hz=valore('CFG_FREQ_MAX_HZ'), rampa_hz_s=valore('CFG_RAMPA_HZ_S'),
                zona_morta=0.0, tick_fermo_inversione=int(valore('CFG_TICK_FERMO_INVERSIONE', 0)))
    assert valore('CFG_ZONA_MORTA') == 0.0
    c, rng = ControlloStepper(p), random.Random(3)
    passi = round((p.frequenza_max_hz - p.frequenza_min_hz) / 100)
    richieste, attese, segno = [], [], 1
    while len(richieste) < 2000:
        if len(richieste) > 6 and rng.random() < 0.6:
            segno = -segno
        u = segno * rng.randint(0, passi) / passi
        for _ in range(rng.randint(1, 6)):
            richieste.append(c.azione_a_hz(u))
            attese.append(c.avanza(u, 0.002).frequenza_hz)
    def array(nome, v):
        return f'static const float {nome}[] = {{' + ','.join(f'{x:.1f}f' for x in v) + '};\n'
    return ('#if CFG_TICK_FERMO_INVERSIONE > 0\n' + array('RAMPA_RICHIESTE', richieste)
            + array('RAMPA_ATTESE', attese) + '#endif\n')


def section(text, name):
    return text.split(f'/* USER CODE BEGIN {name} */', 1)[1].split(f'/* USER CODE END {name} */', 1)[0]

source = (ROOT / 'Core/Src/main.c').read_text()
startup = section(source, '2')
assert startup.index('HAL_TIM_Encoder_Start') < startup.index('Inizializza_Theta()') < startup.index('Inizializza();')
assert 'Inizializza_Theta' not in section(source, 'WHILE')

config_h = (ROOT / 'Core/Inc/config_controllo.h').read_text()
stima_h = (ROOT / 'Core/Inc/stima.h').read_text().replace('#include "config_controllo.h"', '')
stima_c = (ROOT / 'Core/Src/stima.c').read_text().replace('#include "stima.h"', '')
unit = MOCK + config_h + stima_h + stima_c + section(source, 'PD')
unit += rampa_attesa(config_h)
unit += '\nTIM_HandleTypeDef htim2, htim3;\n'
for name in ('PV', 'PFP', '0', '4'):
    unit += section(source, name)
unit += TESTS
with tempfile.TemporaryDirectory(prefix='controllo-swingup-test-') as tmp:
    path = Path(tmp)
    (path / 'test.c').write_text(unit)
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra',
                    '-Wno-unused-parameter', '-Wno-unused-function',
                    str(path / 'test.c'), '-lm', '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
