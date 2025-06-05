/********************************************************************
 * BitDogLab  –  Sintetizador Áudio “duty = amplitude”
 *  • Mic  : GPIO28 (ADC2) — 8 kHz, 12 bits
 *  • PWM : GPIO21 (slice-0A) — 8 bits, 122 kHz
 *  • Botão A (GPIO5)  → Re/Start gravação 5 s
 *  • Botão B (GPIO6)  → Toca buffer 
 ********************************************************************/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

/* ---------- Pinagem -------------------------------------------- */
#define MIC_PIN        28
#define MIC_CH          2
#define PWM_PIN        21
#define BTN_A           5
#define BTN_B           6

/* ---------- Áudio ---------------------------------------------- */
#define FS_HZ         8000u              // 8 kHz
#define SECS             5u
#define N_SAMPLES   (FS_HZ * SECS)
#define US_STEP   (1000000u / FS_HZ)

/* ---------- PWM “DAC” cfg -------------------------------------- */
#define WRAP          255                // 8 bits resolução
#define CLKDIV        4.0f               // 125 MHz/4/256 ≈ 122 kHz

/* ---------- Buffers & Estado ----------------------------------- */
static uint16_t raw_buf[N_SAMPLES];
static volatile uint rec_i = 0, play_i = 0;
typedef enum {IDLE, REC, PLAY} state_t;
static volatile state_t st = IDLE;
static uint pwm_slice;

/* ---------- Callbacks ------------------------------------------ */
static bool rec_cb(struct repeating_timer *t){
    if (rec_i >= N_SAMPLES) return false;
    raw_buf[rec_i++] = adc_read();                     // 0-4095
    return true;
}
static bool play_cb(struct repeating_timer *t){
    if (play_i >= N_SAMPLES) return false;

    /* Converte 12 → 8 bits (descarta 4 LSBs) */
    uint8_t duty = raw_buf[play_i++] >> 4;             // 0-255
    pwm_set_gpio_level(PWM_PIN, duty);                 // duty segue áudio
    return true;
}

/* ---------- Init Periféricos ----------------------------------- */
static void adc_init_mic(void){
    adc_init();
    adc_gpio_init(MIC_PIN);
    adc_select_input(MIC_CH);
}
static void pwm_init_off(void){
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    pwm_slice = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, CLKDIV);
    pwm_config_set_wrap(&c, WRAP);
    pwm_init(pwm_slice, &c, false);       // inicia DESLIGADO
}
static void btn_init(void){
    gpio_init(BTN_A); gpio_set_dir(BTN_A, GPIO_IN); gpio_pull_up(BTN_A);
    gpio_init(BTN_B); gpio_set_dir(BTN_B, GPIO_IN); gpio_pull_up(BTN_B);
}
static inline bool pressed(uint g){ return gpio_get(g)==0; }

/* ============================== main =========================== */
int main(void){
    stdio_init_all();
    adc_init_mic();
    pwm_init_off();
    btn_init();
    sleep_ms(1000);
    printf("Pronto!  A=grava  |  B=play\n");

    struct repeating_timer tmr;

    while (1){

        /* --------- comandos ----------------------------------- */
        if (st == IDLE){
            if (pressed(BTN_A)){
                rec_i = 0;
                add_repeating_timer_us(-(int64_t)US_STEP, rec_cb,NULL,&tmr);
                st = REC;
                printf("gravando 5 s…\n");
                while (pressed(BTN_A)) tight_loop_contents();
            }
            if (pressed(BTN_B) && rec_i == N_SAMPLES){
                play_i = 0;
                pwm_set_enabled(pwm_slice, true);        // liga PWM
                add_repeating_timer_us(-(int64_t)US_STEP, play_cb,NULL,&tmr);
                st = PLAY;
                printf("tocando…\n");
                while (pressed(BTN_B)) tight_loop_contents();
            }
        }

        /* --------- transições -------------------------------- */
        if (st == REC && rec_i >= N_SAMPLES){
            st = IDLE;
            printf("fim gravação\n");
        }
        if (st == PLAY && play_i >= N_SAMPLES){
            pwm_set_enabled(pwm_slice, false);           // silencia
            st = IDLE;
            printf("fim playback\n");
        }

        sleep_ms(4);
    }
}
