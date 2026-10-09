"""Esegue su host il codice USER CODE reale con HAL e rete simulate.

python3 Prova_controllo/tests/test_controllo.py
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
#define AI_NETWORK_IN_1_SIZE 6
#define AI_NETWORK_OUT_1_SIZE 1
#define AI_NETWORK_DATA_ACTIVATIONS_SIZE 1024
static float input_values[6], output_value;
static ai_buffer input_buffer = {input_values}, output_buffer = {&output_value};
static int batches=1, fail_init, during_run;
void HAL_GPIO_EXTI_Callback(uint16_t);
void Controllo_MotorTick(void);
static ai_error ai_network_create_and_init(ai_handle *h, const ai_handle *a, const ai_handle *w) {
    *h=input_values; return (ai_error){fail_init, 0};
}
static ai_buffer *ai_network_inputs_get(ai_handle h, void *n) { return &input_buffer; }
static ai_buffer *ai_network_outputs_get(ai_handle h, void *n) { return &output_buffer; }
static ai_error ai_network_get_error(ai_handle h) { return (ai_error){2, 42}; }
static ai_i32 ai_network_run(ai_handle h, const ai_buffer *i, ai_buffer *o) {
    if (during_run==1) HAL_GPIO_EXTI_Callback(B1_Pin);
    if (during_run==2) tick+=20;
    if (during_run==3) { tick+=40; Controllo_MotorTick(); }
    return batches;
}
'''
TESTS = r'''
static void reset(void) {
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
    corsa_totale_steps=61600;
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
    timer2.CNT+=100; Leggi_theta(); near(theta_rad,PI_F-100*2*PI_F/2400);
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
    timer2.CNT=0; Leggi_theta(); near(theta_rad,PI_F-2*PI_F/2400);

    reset(); start(); tick=20; Rete_Process();
    near(input_values[0],0); near(input_values[1],0); near(input_values[2],-1);
    near(input_values[3],0); near(input_values[4],0); near(input_values[5],0);
    near(frequenza_richiesta_hz,17600); assert(rete_inferenze==1);
    tick=22; Controllo_MotorTick(); assert(stepper_running);
    near(frequenza_rampa_hz,3200); assert(frequenza_applicata_hz>0);
    pulse(); assert(posizione_carrello_steps==30801);
    // Stop B1 immediato, nessun riavvio nei tick successivi; antirimbalzo.
    HAL_GPIO_EXTI_Callback(B1_Pin); assert(!controllo_abilitato && !stepper_running);
    near(frequenza_richiesta_hz,0); near(frequenza_applicata_hz,0);
    HAL_GPIO_EXTI_Callback(B1_Pin); assert(!richiesta_avvio);
    tick=24; Controllo_MotorTick(); assert(!stepper_running);
    pins[B1_Pin]=GPIO_PIN_SET;
    for(int i=0;i<51;i++) Controllo_ButtonTick();
    pins[B1_Pin]=GPIO_PIN_RESET; HAL_GPIO_EXTI_Callback(B1_Pin);
    assert(richiesta_avvio); Controllo_AccettaAvvio(); assert(controllo_abilitato);

    // I sei ingressi, unita' e attraversamento +/-pi senza impulso su omega.
    reset(); start(); tick=20; timer2.CNT=(uint32_t)-2;
    posizione_carrello_steps+=640; frequenza_applicata_hz=-16000;
    Rete_Process();
    // Prima lettura: lo stimatore parte dalla misura, velocita' nulle.
    near(input_values[0],0.0096f/0.462f); near(input_values[3],0);
    near(input_values[4],0); near(input_values[5],-0.5f);
    near(stima.theta,-PI_F+2*2*PI_F/2400);
    // Lo stimatore integra la velocita' STEP programmata ogni 2 ms in SysTick;
    // le letture rumorose dell'angolo vengono attenuate (L_THETA=0.2).
    reset(); start(); tick=20; Rete_Process();
    stepper_running=1; frequenza_applicata_hz=3200;
    for (int i=0;i<10;i++){ tick+=2; Controllo_StimaTick(); }
    near(stima.x,10*0.002f*3200*METRI_PER_STEP); near(stima.velocita,0.048f);
    timer2.CNT+=(uint32_t)(-24); tick=40; Rete_Process();
    near(input_values[3],0.1f);
    near(input_values[0],stima.x/0.462f);
    // 24 conteggi (3,6 gradi) in 20 ms: la derivata darebbe omega=3,1 rad/s,
    // lo stimatore ne prende solo una frazione.
    assert(input_values[4]>0 && input_values[4]*10<0.5f*(24*2*PI_F/2400)/0.02f);
    // A controllo fermo: v=0 e x agganciata al conteggio STEP.
    Controllo_Disabilita(); float xprima=stima.x;
    for (int i=0;i<5;i++){ tick+=2; Controllo_StimaTick(); }
    near(stima.x,xprima); near(stima.velocita,0);
    tick=60; Rete_Process(); near(stima.x,0);

    // Uscita zero e saturazione, senza seconda tanh.
    reset(); start(); tick=20; Rete_Process();
    output_value=0; tick=40; Rete_Process(); near(frequenza_richiesta_hz,0);
    output_value=-2; tick=60; Rete_Process(); near(frequenza_richiesta_hz,-32000);

    // Decelerazione e inversione solo dopo un impulso completo e setup DIR.
    reset(); start(); frequenza_richiesta_hz=6400;
    tick=2; Controllo_MotorTick(); tick=4; Controllo_MotorTick();
    near(frequenza_rampa_hz,6400);
    frequenza_richiesta_hz=-6400;
    tick=6; Controllo_MotorTick(); near(frequenza_rampa_hz,3200);
    tick=8; Controllo_MotorTick(); assert(stop_al_fronte_step && stepper_running);
    pulse(); assert(!stepper_running); near(frequenza_applicata_hz,0);
    tick=10; Controllo_MotorTick(); assert(!stepper_running && stepper_direction==DIR_FORWARD);
    tick=12; Controllo_MotorTick(); assert(stepper_running && frequenza_applicata_hz<0);
    int32_t before=posizione_carrello_steps; pulse(); assert(posizione_carrello_steps==before-1);
    frequenza_richiesta_hz=0;
    tick=14; Controllo_MotorTick(); tick=16; Controllo_MotorTick(); pulse();
    assert(!stepper_running); near(frequenza_applicata_hz,0);

    // B1 durante l'inferenza non permette di pubblicare il vecchio comando.
    reset(); start(); during_run=1; tick=20; Rete_Process();
    assert(!controllo_abilitato); near(frequenza_richiesta_hz,0); assert(rete_inferenze==0);
    // Errori rete, valori non finiti e deadline fermano il controllo.
    reset(); start(); batches=0; tick=20; Rete_Process();
    assert(controllo_errore==2 && !rete_pronta && !controllo_abilitato);
    reset(); start(); output_value=NAN; tick=20; Rete_Process();
    assert(controllo_errore==3 && !controllo_abilitato);
    reset(); start(); during_run=2; tick=20; Rete_Process();
    assert(controllo_errore==5 && !controllo_abilitato);
    reset(); start(); during_run=3; tick=20; Rete_Process();
    assert(controllo_errore==5 && !controllo_abilitato);
    reset(); start(); tick=40; Controllo_MotorTick();
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
    reset(); start(); fail_pwm=1; frequenza_richiesta_hz=3200; tick=2; Controllo_MotorTick();
    assert(controllo_errore==6 && !controllo_abilitato);
    reset(); fail_init=1; assert(!Rete_Init());
    // Tick wraparound.
    reset(); tick=UINT32_MAX-10; start(); tick=9; ultimo_campione_ms=UINT32_MAX-10;
    Rete_Process(); assert(rete_inferenze==1 && controllo_abilitato);
    puts("OK: calibrazione theta pre-homing, ingressi, azioni, rampa, inversione, B1, fault, deadline e wraparound");
}
'''

def section(text, name):
    return text.split(f'/* USER CODE BEGIN {name} */', 1)[1].split(f'/* USER CODE END {name} */', 1)[0]

source = (ROOT / 'Core/Src/main.c').read_text()
startup = section(source, '2')
assert startup.index('HAL_TIM_Encoder_Start') < startup.index('Inizializza_Theta()') < startup.index('Inizializza();')
assert 'Inizializza_Theta' not in section(source, 'WHILE')

stima_h = (ROOT / 'Core/Inc/stima.h').read_text()
stima_c = (ROOT / 'Core/Src/stima.c').read_text().replace('#include "stima.h"', '')
unit = MOCK + stima_h + stima_c + section(source, 'PD')
unit += '\nTIM_HandleTypeDef htim2, htim3;\n'
for name in ('PV', 'PFP', '0', '4'):
    unit += section(source, name)
unit += TESTS
with tempfile.TemporaryDirectory(prefix='prova-controllo-test-') as tmp:
    path = Path(tmp)
    (path / 'test.c').write_text(unit)
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra',
                    '-Wno-unused-parameter', '-Wno-unused-function',
                    str(path / 'test.c'), '-lm', '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
