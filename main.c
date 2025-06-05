/********************************************************************
 * BitDogLab – Botão A grava / Botão B reproduz (duty = amostra GANHADA)
 ********************************************************************/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

/* ------------ pinos -------------------------------------------- */
#define MIC_PIN        28
#define MIC_CH          2
#define PWM_PIN        21
#define BTN_A           5
#define BTN_B           6

/* ------------ áudio -------------------------------------------- */
#define FS_HZ         8000u
#define SECS             5u
#define N_SAMPLES   (FS_HZ * SECS)
#define US_STEP   (1000000u / FS_HZ)

/* ------------ PWM --------------------------------------------- */
#define WRAP          255
#define CLKDIV          4.0f
#define PWM_CLK   (125000000/CLKDIV)

/* ------------ estado / buffers -------------------------------- */
typedef enum {IDLE, REC, PLAY} state_t;
static volatile state_t st = IDLE;
static uint16_t raw_buf[N_SAMPLES];
static volatile uint rec_i = 0, play_i = 0;
static uint pwm_slice;

/* ------------ helpers ----------------------------------------- */
static inline bool pressed(uint g){ return gpio_get(g)==0; }

/* ------------ callbacks --------------------------------------- */
static bool rec_cb(struct repeating_timer *t){
    if (rec_i >= N_SAMPLES) return false;
    raw_buf[rec_i++] = adc_read();
    return true;
}
static bool play_cb(struct repeating_timer *t){
    if (play_i >= N_SAMPLES) return false;

    int32_t diff = (int32_t)raw_buf[play_i++] - 2048;    // –2048..+2047
    const int32_t GAIN = 12;                             
    int32_t duty = (diff * GAIN) / 16 + 128;             // ganho & centro
    if (duty < 0)   duty = 0;
    if (duty > 255) duty = 255;

    pwm_set_gpio_level(PWM_PIN, (uint8_t)duty);
    return true;
}

/* ------------ init funcs -------------------------------------- */
static void adc_init_mic(void){
    adc_init();
    adc_gpio_init(MIC_PIN);
    adc_select_input(MIC_CH);
}
static void pwm_init_off(void){
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    pwm_slice = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, CLKDIV);
    pwm_config_set_wrap(&cfg, WRAP);
    pwm_init(pwm_slice, &cfg, false);
}
static void btn_init(void){
    gpio_init(BTN_A); gpio_set_dir(BTN_A, GPIO_IN); gpio_pull_up(BTN_A);
    gpio_init(BTN_B); gpio_set_dir(BTN_B, GPIO_IN); gpio_pull_up(BTN_B);
}

/* =============================== main ========================== */
int main(void){
    stdio_init_all();
    adc_init_mic();
    pwm_init_off();
    btn_init();
    sleep_ms(1000);

    struct repeating_timer tmr;
    printf("A = grava  |  B = play\n");

    while (1){
        if (st == IDLE){
            if (pressed(BTN_A)){
                rec_i = 0;
                add_repeating_timer_us(-(int64_t)US_STEP, rec_cb,NULL,&tmr);
                st = REC;
                printf("gravando...\n");
                while (pressed(BTN_A)) tight_loop_contents();
            }
            if (pressed(BTN_B) && rec_i == N_SAMPLES){
                play_i = 0;
                pwm_set_enabled(pwm_slice, true);
                add_repeating_timer_us(-(int64_t)US_STEP, play_cb,NULL,&tmr);
                st = PLAY;
                printf("tocando...\n");
                while (pressed(BTN_B)) tight_loop_contents();
            }
        }
        if (st == REC && rec_i >= N_SAMPLES){
            st = IDLE;
            printf("fim gravação\n");
        }
        if (st == PLAY && play_i >= N_SAMPLES){
            pwm_set_enabled(pwm_slice, false);
            st = IDLE;
            printf("fim playback\n");
        }
        sleep_ms(4);
    }
}
