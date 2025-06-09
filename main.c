/********************************************************************
 * BitDogLab – Gravador 15 s  +  Player  +  OLED (scope)  +  LED
 * 
 ********************************************************************/
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "include/ssd1306.h"
#include "include/ssd1306_i2c.h"
#include "include/ssd1306_font.h"

#define OLED_W 128
#define OLED_H 64
static ssd1306_t disp;

/* ---------------- pinos ---------------- */
#define MIC_PIN  28
#define MIC_CH    2
#define PWM_PIN  21
#define BTN_A     5
#define BTN_B     6
#define LED_R_PIN 13
#define LED_G_PIN 11

/* -------------- áudio ------------------ */
#define FS_HZ   8000u
#define SECS       5u
#define N_SAMPLES (FS_HZ*SECS)
#define US_STEP (1000000u/FS_HZ)

/* -------------- PWM -------------------- */
#define WRAP   1023
#define CLKDIV   4.0f

typedef enum {IDLE, REC, PLAY} state_t;
static volatile state_t st = IDLE;

/* buffers */
static uint16_t buf[N_SAMPLES];
static volatile uint rec_i = 0, play_i = 0;
static uint slice;

/* ---- scope buffer (1 byte por coluna) ---- */
static uint8_t  scope_buf[OLED_W];
static uint8_t  col_write = 0;   /* próxima coluna a escrever */
static uint8_t  col_draw  = 0;   /* próxima coluna a desenhar */

/* ========= LED ========= */
static inline void led_rgb(bool r,bool g){ gpio_put(LED_R_PIN,r); gpio_put(LED_G_PIN,g); }
#define led_idle() led_rgb(0,0)
#define led_rec()  led_rgb(1,0)
#define led_play() led_rgb(0,1)

/* ========= OLED text helpers ========= */
static void oled_setup(void){
    i2c_init(i2c1,400000);
    gpio_set_function(14,GPIO_FUNC_I2C);
    gpio_set_function(15,GPIO_FUNC_I2C);
    gpio_pull_up(14); gpio_pull_up(15);
    disp.external_vcc=false;
    ssd1306_init(&disp,OLED_W,OLED_H,0x3C,i2c1);
}
static void oled_center(const char *s,int y){
    int x=(OLED_W-(int)strlen(s)*6)/2;
    ssd1306_draw_string(&disp,x,y,1,s);
}
static void oled_idle(void){
    ssd1306_clear(&disp);
    oled_center("Botao A -> gravar",28);
    ssd1306_show(&disp);
}
static void oled_recording(void){
    ssd1306_clear(&disp);
    oled_center("Gravando audio..",28);
    ssd1306_show(&disp);
}
static void oled_recorded(void){
    ssd1306_clear(&disp);
    oled_center("Audio gravado",20);
    oled_center("Botao B -> reproduzir",32);
    ssd1306_show(&disp);
}

/* ============ helpers ============ */
static inline bool pressed(uint g){ return gpio_get(g)==0; }
static inline uint16_t abs16(int32_t v){ return v<0? -v : v; }

/* ============ scope drawing (main-loop) ============ */
static void draw_scope_col(uint8_t x,uint8_t mag){   /* mag 0-31 */
    for(int y=0;y<OLED_H;y++) ssd1306_clear_pixel(&disp,x,y);
    for(int dy=0;dy<=mag;dy++){
        ssd1306_draw_pixel(&disp,x, OLED_H/2 + dy);
        ssd1306_draw_pixel(&disp,x, OLED_H/2 - dy);
    }
}
static void scope_refresh(void){
    if(st!=PLAY) return;
    while(col_draw != col_write){
        uint8_t h = scope_buf[col_draw];
        draw_scope_col(col_draw,h);
        col_draw = (col_draw+1)&0x7F;
    }
    ssd1306_show(&disp);
}

/* ============ callbacks ============ */
static bool rec_cb(struct repeating_timer *t){
    if(rec_i>=N_SAMPLES) return false;
    buf[rec_i++] = adc_read();
    return true;
}
static bool play_cb(struct repeating_timer *t){
    if(play_i>=N_SAMPLES) return false;

    /* ---- PWM (igual) ---- */
    int32_t diff = (int32_t)buf[play_i++] - 2048;
    int32_t x = (diff*12)/2 + 512;
    static int32_t y=512;
    y += (x - y)>>2;
    if(y<0)y=0; else if(y>1023)y=1023;
    pwm_set_gpio_level(PWM_PIN,(uint16_t)y);

    /* ---- scope (mínimo) ---- */
    uint16_t a = abs16(diff);        // 0-2047
    if(a > scope_buf[col_write]) scope_buf[col_write]= a>>6;  /* 0-31 */
    if(++col_write==OLED_W){ col_write=0; }
    return true;
}

/* ============ init hw ============ */
static void init_hw(void){
    stdio_init_all();
    oled_setup(); oled_idle();

    adc_init(); adc_gpio_init(MIC_PIN); adc_select_input(MIC_CH);

    gpio_set_function(PWM_PIN,GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config c=pwm_get_default_config();
    pwm_config_set_clkdiv(&c,CLKDIV);
    pwm_config_set_wrap(&c,WRAP);
    pwm_init(slice,&c,false);

    gpio_init(BTN_A); gpio_set_dir(BTN_A,GPIO_IN); gpio_pull_up(BTN_A);
    gpio_init(BTN_B); gpio_set_dir(BTN_B,GPIO_IN); gpio_pull_up(BTN_B);

    gpio_init(LED_R_PIN); gpio_set_dir(LED_R_PIN,GPIO_OUT);
    gpio_init(LED_G_PIN); gpio_set_dir(LED_G_PIN,GPIO_OUT);
    led_idle();
}

/* =========================== main ============================= */
int main(void)
{
    init_hw();   sleep_ms(400);
    struct repeating_timer tim;

    while(1){
        /* IDLE → REC */
        if(st==IDLE && pressed(BTN_A)){
            rec_i=0;
            add_repeating_timer_us(-(int64_t)US_STEP,rec_cb,NULL,&tim);
            st=REC; oled_recording(); led_rec();
            while(pressed(BTN_A)) tight_loop_contents();
        }

        /* IDLE → PLAY */
        if(st==IDLE && pressed(BTN_B) && rec_i==N_SAMPLES){
            play_i=0; col_write=col_draw=0;
            memset(scope_buf,0,sizeof scope_buf);
            ssd1306_clear(&disp); ssd1306_show(&disp);
            pwm_set_enabled(slice,true);
            add_repeating_timer_us(-(int64_t)US_STEP,play_cb,NULL,&tim);
            st=PLAY; led_play();
            while(pressed(BTN_B)) tight_loop_contents();
        }

        /* REC terminou */
        if(st==REC && rec_i>=N_SAMPLES){
            st=IDLE; oled_recorded(); led_idle();
        }

        /* PLAY terminou */
        if(st==PLAY && play_i>=N_SAMPLES){
            pwm_set_enabled(slice,false);
            st=IDLE; oled_idle(); led_idle();
        }

        scope_refresh();          // desenha quando tiver folga
        sleep_ms(4);
    }
}