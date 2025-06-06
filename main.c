/********************************************************************
 * BitDogLab ▸ Botão A grava 5 s · Botão B reproduz
 * Ajustes:  • PWM 10 bits (wrap = 1023   →   degrau 4× menor)
 *           • IIR 1ª ordem   y = y + (x−y)/4   (suaviza)
 *           • GAIN 12×        (mantém ±128 níveis úteis)
 ********************************************************************/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

/* —— pinos ——————————————————————————————————————————————— */
#define MIC_PIN   28     // ADC2
#define MIC_CH     2
#define PWM_PIN   21     // Buzzer A  ➜ filtro RC / amp
#define BTN_A      5
#define BTN_B      6

/* —— áudio ——————————————————————————————————————————————— */
#define FS_HZ        8000u
#define SECS            15u
#define N_SAMPLES  (FS_HZ * SECS)
#define US_STEP  (1000000u / FS_HZ)

/* —— PWM: 10 bits @ 30 kHz ———————————————————————————— */
#define WRAP        1023          // 10 bits
#define CLKDIV        4.0f        // 125 MHz/4/1024 ≈ 30.5 kHz

/* —— buffers / estado ——————————————————————————————— */
typedef enum {IDLE, REC, PLAY} state_t;
static volatile state_t st = IDLE;
static uint16_t buf[N_SAMPLES];
static volatile uint rec_i = 0, play_i = 0;
static uint slice;

/* —— helpers ——————————————————————————————————————————— */
static inline bool pressed(uint g){ return gpio_get(g) == 0; }

/* —— callbacks —————————————————————————————————————————— */
static bool rec_cb(struct repeating_timer *t){
    if (rec_i >= N_SAMPLES) return false;
    buf[rec_i++] = adc_read();          /* 0-4095 */
    return true;
}

static bool play_cb(struct repeating_timer *t){
    if (play_i >= N_SAMPLES) return false;

    /* 1. diferença centrada */
    int32_t diff = (int32_t)buf[play_i++] - 2048;   // –2048…+2047

    /* 2. ganho digital (≈×12) para ocupar ±128 níveis */
    int32_t x = (diff * 12) / 2 + 512;              // 0-1023 alvo

    /* 3. filtro IIR  y = y + (x-y)/4  (α = 0.25)  */
    static int32_t y = 512;
    y += (x - y) >> 2;

    /* 4. clamp & envia */
    if (y < 0) y = 0; else if (y > 1023) y = 1023;
    pwm_set_gpio_level(PWM_PIN, (uint16_t)y);
    return true;
}

/* —— inits —————————————————————————————————————————————— */
static void adc_init_mic(void){
    adc_init();
    adc_gpio_init(MIC_PIN);
    adc_select_input(MIC_CH);
}
static void pwm_init_off(void){
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(PWM_PIN);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, CLKDIV);
    pwm_config_set_wrap(&cfg, WRAP);
    pwm_init(slice, &cfg, false);       // começa OFF
}
static void btn_init(void){
    gpio_init(BTN_A); gpio_set_dir(BTN_A, GPIO_IN); gpio_pull_up(BTN_A);
    gpio_init(BTN_B); gpio_set_dir(BTN_B, GPIO_IN); gpio_pull_up(BTN_B);
}

/* —— main ——————————————————————————————————————————————— */
int main(){
    stdio_init_all();
    adc_init_mic(); pwm_init_off(); btn_init();
    sleep_ms(1000);
    printf("A=grava  |  B=toca\n");

    struct repeating_timer t;

    while (1){
        /* idle → grava */
        if (st == IDLE && pressed(BTN_A)){
            rec_i = 0;
            add_repeating_timer_us(-(int64_t)US_STEP, rec_cb,NULL,&t);
            st = REC; printf("gravando...\n");
            while (pressed(BTN_A)) tight_loop_contents();
        }

        /* idle → play (se já gravou) */
        if (st == IDLE && pressed(BTN_B) && rec_i == N_SAMPLES){
            play_i = 0; pwm_set_enabled(slice, true);
            add_repeating_timer_us(-(int64_t)US_STEP, play_cb,NULL,&t);
            st = PLAY; printf("tocando...\n");
            while (pressed(BTN_B)) tight_loop_contents();
        }

        /* grava terminou */
        if (st == REC && rec_i >= N_SAMPLES){
            st = IDLE; printf("fim gravação\n");
        }

        /* play terminou */
        if (st == PLAY && play_i >= N_SAMPLES){
            pwm_set_enabled(slice, false);
            st = IDLE; printf("fim playback\n");
        }

        sleep_ms(4);
    }
}
