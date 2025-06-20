/********************************************************************
 * BitDogLab – Gravador + Player + OLED (scope) + LED + Segundos dinâmicos
 * (agora com calibração automática do offset do microfone)
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

/* ---------- Configurações do display OLED ---------- */
#define OLED_W 128
#define OLED_H 64
static ssd1306_t disp; // Estrutura do display

/* ---------- Definição de pinos ---------- */
#define MIC_PIN   28      // Pino do microfone (canal ADC2)
#define MIC_CH     2
#define JOY_X_PIN  26      // Pino do joystick eixo X (canal ADC0)
#define JOY_X_CH    0
#define PWM_PIN    21      // Pino de saída de áudio (PWM)
#define BTN_A       5      // Botão A - iniciar gravação
#define BTN_B       6      // Botão B - reproduzir
#define LED_R_PIN  13      // LED vermelho (indica gravação)
#define LED_G_PIN  11      // LED verde (indica reprodução)

/* ---------- Configurações de áudio ---------- */
#define FS_HZ      8000u                   // Frequência de amostragem: 8kHz
#define MAX_SECS      15u                  // Tempo máximo gravável
#define MIN_SECS       1u                  // Tempo mínimo gravável
#define DEF_SECS       5u                  // Valor padrão inicial
#define BUF_SAMPLES (FS_HZ * MAX_SECS)     // Tamanho total do buffer de áudio
#define US_STEP (1000000u / FS_HZ)         // Intervalo entre amostras (125 us)

/* ---------- PWM ---------- */
#define WRAP   1023                        // Resolução do PWM
#define CLKDIV 4.0f                        // Clock divisor para PWM ~30kHz

/* ---------- Estados do sistema ---------- */
typedef enum {IDLE, REC, PLAY} state_t;
static volatile state_t st = IDLE;        // Estado atual do sistema

/* ---------- Buffers e controle ---------- */
static uint16_t buf[BUF_SAMPLES];         // Buffer de áudio
static volatile uint rec_i = 0, play_i = 0; // Índices de gravação e reprodução
static uint slice;                         // PWM slice associado ao pino

/* ---------- Buffer para escopo OLED ---------- */
static uint8_t scope_buf[OLED_W];         // Barras de histograma
static uint8_t col_write = 0, col_draw = 0;

/* ---------- Tempo configurado dinamicamente ---------- */
static uint secs = DEF_SECS;                  // Tempo selecionado (em segundos)
static uint target_samples = DEF_SECS * FS_HZ; // Total de amostras a gravar/reproduzir

/* ---------- Calibração do microfone ---------- */
static uint16_t mic_bias = 2048; // Offset dinâmico para centralizar o sinal

/* ---------- Controle de LEDs ---------- */
static inline void led_rgb(bool r, bool g) { gpio_put(LED_R_PIN, r); gpio_put(LED_G_PIN, g); }
#define led_idle() led_rgb(0,0)
#define led_rec()  led_rgb(1,0)
#define led_play() led_rgb(0,1)

/* ---------- Inicializa o OLED ---------- */
static void oled_setup(void){
    i2c_init(i2c1, 400000);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_pull_up(14); gpio_pull_up(15);
    disp.external_vcc = false;
    ssd1306_init(&disp, OLED_W, OLED_H, 0x3C, i2c1);
}

/* ---------- Exibe texto centralizado ---------- */
static void oled_center(const char *s, int y){
    int x = (OLED_W - (int)strlen(s)*6)/2;
    ssd1306_draw_string(&disp, x, y, 1, s);
}

/* ---------- Telas ---------- */
static void oled_idle(void){
    char line[24];
    ssd1306_clear(&disp);
    oled_center("Botao A -> gravar", 22);
    sprintf(line, "Segundos: %u", secs);
    oled_center(line, 34);
    ssd1306_show(&disp);
}
static void oled_recording(void){
    ssd1306_clear(&disp);
    oled_center("Gravando audio..", 28);
    ssd1306_show(&disp);
}
static void oled_recorded(void){
    ssd1306_clear(&disp);
    oled_center("Audio gravado", 20);
    oled_center("Botao B -> reproduzir", 32);
    ssd1306_show(&disp);
}

/* ---------- Funções auxiliares ---------- */
static inline bool pressed(uint g){ return gpio_get(g)==0; }
static inline uint16_t abs16(int32_t v){ return v < 0 ? -v : v; }

/* ---------- Desenha colunas do escopo OLED ---------- */
static void draw_scope_col(uint8_t x, uint8_t mag){
    for(int y=0; y<OLED_H; y++) ssd1306_clear_pixel(&disp, x, y);
    for(int dy=0; dy<=mag; dy++){
        ssd1306_draw_pixel(&disp, x, OLED_H/2 + dy);
        ssd1306_draw_pixel(&disp, x, OLED_H/2 - dy);
    }
}
static void scope_refresh(void){
    if(st != PLAY) return;
    while(col_draw != col_write){
        draw_scope_col(col_draw, scope_buf[col_draw]);
        col_draw = (col_draw + 1) & 0x7F;
    }
    ssd1306_show(&disp);
}

/* ---------- Calibração do offset do microfone ---------- */
static void calibrate_mic_bias(void){
    uint32_t acc = 0;
    for(int i = 0; i < 200; i++){
        adc_select_input(MIC_CH);
        acc += adc_read();
        sleep_us(50);
    }
    mic_bias = (uint16_t)(acc / 200);
}

/* ---------- Callback de gravação ---------- */
static bool rec_cb(struct repeating_timer *t){
    if(rec_i >= target_samples) return false;
    adc_select_input(MIC_CH);
    buf[rec_i++] = adc_read(); // Armazena a amostra bruta
    return true;
}

/* ---------- Callback de reprodução ---------- */
static bool play_cb(struct repeating_timer *t){
    if(play_i >= target_samples) return false;
    int32_t diff = (int32_t)buf[play_i++] - mic_bias;
    int32_t x = (diff*12)/2 + 512; static int32_t y = 512;
    y += (x - y) >> 2; // Filtro IIR
    if(y<0) y=0; else if(y>1023) y=1023;
    pwm_set_gpio_level(PWM_PIN, (uint16_t)y);

    uint16_t a = abs16(diff);
    if(a > scope_buf[col_write]) scope_buf[col_write] = a >> 6;
    if(++col_write == OLED_W) col_write = 0;
    return true;
}

/* ---------- Inicializa todo o hardware ---------- */
static void init_hw(void){
    stdio_init_all();
    oled_setup(); oled_idle();

    adc_init();
    adc_gpio_init(MIC_PIN);
    adc_gpio_init(JOY_X_PIN);

    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, CLKDIV);
    pwm_config_set_wrap(&c, WRAP);
    pwm_init(slice, &c, false);

    gpio_init(BTN_A); gpio_set_dir(BTN_A, GPIO_IN); gpio_pull_up(BTN_A);
    gpio_init(BTN_B); gpio_set_dir(BTN_B, GPIO_IN); gpio_pull_up(BTN_B);

    gpio_init(LED_R_PIN); gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_G_PIN); gpio_set_dir(LED_G_PIN, GPIO_OUT);
    led_idle();
}

/* ---------- Controle do joystick (ajusta segundos) ---------- */
static void joystick_update(void){
    if(st != IDLE) return;
    adc_select_input(JOY_X_CH);
    uint16_t raw = adc_read();
    static uint8_t joy_state = 0;

    const uint16_t TH_LEFT  = 1000;
    const uint16_t TH_RIGHT = 3000;

    if(raw < TH_LEFT && joy_state != 1){
        if(secs > MIN_SECS){ secs--; target_samples = secs * FS_HZ; oled_idle(); }
        joy_state = 1;
    }else if(raw > TH_RIGHT && joy_state != 2){
        if(secs < MAX_SECS){ secs++; target_samples = secs * FS_HZ; oled_idle(); }
        joy_state = 2;
    }else if(raw >= TH_LEFT && raw <= TH_RIGHT){
        joy_state = 0;
    }
}

/* ---------- LOOP PRINCIPAL ---------- */
int main(void){
    init_hw(); sleep_ms(300);
    struct repeating_timer tim;

    while(1){
        joystick_update();

        // Inicia gravação
        if(st == IDLE && pressed(BTN_A)){
            calibrate_mic_bias();
            rec_i = 0;
            add_repeating_timer_us(-(int64_t)US_STEP, rec_cb, NULL, &tim);
            st = REC; oled_recording(); led_rec();
            while(pressed(BTN_A)) tight_loop_contents();
        }

        // Inicia reprodução
        if(st == IDLE && pressed(BTN_B) && rec_i == target_samples){
            play_i = 0; col_write = col_draw = 0;
            memset(scope_buf, 0, sizeof scope_buf);
            ssd1306_clear(&disp); ssd1306_show(&disp);
            pwm_set_enabled(slice, true);
            add_repeating_timer_us(-(int64_t)US_STEP, play_cb, NULL, &tim);
            st = PLAY; led_play();
            while(pressed(BTN_B)) tight_loop_contents();
        }

        // Fim da gravação
        if(st == REC && rec_i >= target_samples){
            st = IDLE; oled_recorded(); led_idle();
        }

        // Fim da reprodução
        if(st == PLAY && play_i >= target_samples){
            pwm_set_enabled(slice, false);
            st = IDLE; oled_idle(); led_idle();
        }

        scope_refresh();
        sleep_ms(4);
    }
}
