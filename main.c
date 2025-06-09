/********************************************************************
 * BitDogLab – Gravador 15 s + Player + OLED + LED RGB
 ********************************************************************/
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"

/* ---------- SSD1306 ------------------------------------------- */
#include "include/ssd1306.h"
#include "include/ssd1306_i2c.h"
#include "include/ssd1306_font.h"

#define OLED_W 128
#define OLED_H 64
static ssd1306_t disp;

/* ---------- pinos --------------------------------------------- */
#define MIC_PIN   28
#define MIC_CH     2
#define PWM_PIN   21

#define BTN_A      5
#define BTN_B      6

/* ★ LED RGB ----------------------------------------------------- */
#define LED_R_PIN 13     
#define LED_G_PIN 11


/* ---------- áudio --------------------------------------------- */
#define FS_HZ    8000u
#define SECS        15u
#define N_SAMPLES (FS_HZ * SECS)
#define US_STEP (1000000u / FS_HZ)

/* ---------- PWM 10 bits @30 kHz ------------------------------- */
#define WRAP   1023
#define CLKDIV   4.0f

/* ---------- estados ------------------------------------------- */
typedef enum {IDLE, REC, PLAY} state_t;
static volatile state_t st = IDLE;

/* ---------- buffers ------------------------------------------- */
static uint16_t buf[N_SAMPLES];
static volatile uint rec_i = 0, play_i = 0;
static uint slice;

/* ========== LED helpers (digital on/off) ====================== */
static inline void led_rgb(bool r, bool g, bool b){
    gpio_put(LED_R_PIN, r);
    gpio_put(LED_G_PIN, g);
    
}
static inline void led_idle(void){     led_rgb(0,0,0); }  // apagado
static inline void led_rec(void){      led_rgb(1,0,0); }  // vermelho
static inline void led_play(void){     led_rgb(0,1,0); }  // verde

/* ========== OLED helpers ==================== */
static void oled_setup(void){
    i2c_init(i2c1,400000);
    gpio_set_function(14,GPIO_FUNC_I2C);
    gpio_set_function(15,GPIO_FUNC_I2C);
    gpio_pull_up(14); gpio_pull_up(15);
    disp.external_vcc=false;
    ssd1306_init(&disp,OLED_W,OLED_H,0x3C,i2c1);
    ssd1306_clear(&disp); ssd1306_show(&disp);
}
static void oled_center(const char *txt,int y){
    int x=(OLED_W-strlen(txt)*6)/2;
    ssd1306_draw_string(&disp,x,y,1,txt);
}
static void oled_idle_screen(void){
    ssd1306_clear(&disp);
    oled_center("Botao A -> gravar", 28);        
    ssd1306_show(&disp);
}
static void oled_recording(void){
    ssd1306_clear(&disp);
    oled_center("Gravando audio..", 28);
    ssd1306_show(&disp);
}
static void oled_recorded(void){
    ssd1306_clear(&disp);
    oled_center("Audio gravado",          20);
    oled_center("Botao B -> reproduzir",  32);   
    ssd1306_show(&disp);
}
static void oled_playing(void){
    ssd1306_clear(&disp);
    oled_center("Tocando...", 28);
    ssd1306_show(&disp);
}

/* ========== helpers botão ===================================== */
static inline bool pressed(uint g){ return gpio_get(g)==0; }

/* ========== callbacks ========================================= */
static bool rec_cb(struct repeating_timer *t){
    if(rec_i>=N_SAMPLES) return false;
    buf[rec_i++] = adc_read();
    return true;
}
static bool play_cb(struct repeating_timer *t){
    if(play_i>=N_SAMPLES) return false;
    int32_t diff = (int32_t)buf[play_i++] - 2048;
    int32_t x = (diff*12)/2 + 512;
    static int32_t y = 512;
    y += (x - y)>>2;
    if(y<0) y=0; else if(y>1023) y=1023;
    pwm_set_gpio_level(PWM_PIN,(uint16_t)y);
    return true;
}

/* ========== inits ============================================= */
static void adc_init_mic(void){
    adc_init(); adc_gpio_init(MIC_PIN); adc_select_input(MIC_CH);
}
static void pwm_init_off(void){
    gpio_set_function(PWM_PIN,GPIO_FUNC_PWM);
    slice=pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config c=pwm_get_default_config();
    pwm_config_set_clkdiv(&c,CLKDIV);
    pwm_config_set_wrap(&c,WRAP);
    pwm_init(slice,&c,false);
}
static void btn_init(void){
    gpio_init(BTN_A); gpio_set_dir(BTN_A,GPIO_IN); gpio_pull_up(BTN_A);
    gpio_init(BTN_B); gpio_set_dir(BTN_B,GPIO_IN); gpio_pull_up(BTN_B);
}
static void led_init(void){
    gpio_init(LED_R_PIN); gpio_set_dir(LED_R_PIN,GPIO_OUT);
    gpio_init(LED_G_PIN); gpio_set_dir(LED_G_PIN,GPIO_OUT);
    led_idle();
}

/* ============================ main ============================ */
int main(void)
{
    stdio_init_all();
    oled_setup();
    adc_init_mic();
    pwm_init_off();
    btn_init();
    led_init();

    sleep_ms(500);
    oled_idle_screen();

    struct repeating_timer tim;

    while(true){
        /* ----- IDLE → REC ------------------------------------- */
        if(st==IDLE && pressed(BTN_A)){
            rec_i=0;
            add_repeating_timer_us(-(int64_t)US_STEP,rec_cb,NULL,&tim);
            st = REC;  oled_recording();  led_rec();
            while(pressed(BTN_A)) tight_loop_contents();
        }

        /* ----- IDLE → PLAY ------------------------------------ */
        if(st==IDLE && pressed(BTN_B) && rec_i==N_SAMPLES){
            play_i=0; pwm_set_enabled(slice,true);
            add_repeating_timer_us(-(int64_t)US_STEP,play_cb,NULL,&tim);
            st = PLAY; oled_playing(); led_play();
            while(pressed(BTN_B)) tight_loop_contents();
        }

        /* ----- REC terminou ----------------------------------- */
        if(st==REC && rec_i>=N_SAMPLES){
            st = IDLE; oled_recorded(); led_idle();
        }

        /* ----- PLAY terminou ---------------------------------- */
        if(st==PLAY && play_i>=N_SAMPLES){
            pwm_set_enabled(slice,false);
            st = IDLE; oled_idle_screen(); led_idle();
        }

        sleep_ms(4);
    }
}