/********************************************************************
 * BitDogLab – Gravador 15 s  +  Player  +  OLED (scope)  +  LED
 * 
 * Funcionalidades:
 * - Grava 5 segundos de áudio usando o microfone (ADC)
 * - Reproduz o áudio usando PWM (via buzzer)
 * - Exibe um "osciloscópio" com a forma de onda no OLED
 * - Usa LED vermelho durante gravação, verde durante reprodução
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

/* ==== Display OLED ==== */
#define OLED_W 128
#define OLED_H 64
static ssd1306_t disp;  // objeto do display

/* ==== Mapeamento de Pinos ==== */
#define MIC_PIN  28     // pino do microfone (ADC2)
#define MIC_CH    2     // canal ADC correspondente
#define PWM_PIN  21     // pino do buzzer (saída PWM)
#define BTN_A     5     // botão A (gravar)
#define BTN_B     6     // botão B (reproduzir)
#define LED_R_PIN 13    // LED vermelho
#define LED_G_PIN 11    // LED verde

/* ==== Parâmetros do Áudio ==== */
#define FS_HZ   8000u                      // taxa de amostragem (8 kHz)
#define SECS       5u                      // duração da gravação (5s)
#define N_SAMPLES (FS_HZ * SECS)           // total de amostras
#define US_STEP (1000000u / FS_HZ)         // intervalo entre amostras (em micros)

/* ==== Configuração do PWM ==== */
#define WRAP   1023        // resolução de 10 bits
#define CLKDIV   4.0f      // divisor do clock -> ~30kHz

/* ==== Estados do sistema ==== */
typedef enum {IDLE, REC, PLAY} state_t;
static volatile state_t st = IDLE;         // estado atual

/* ==== Buffers de Áudio ==== */
static uint16_t buf[N_SAMPLES];            // armazenamento do áudio
static volatile uint rec_i = 0, play_i = 0;// índices de gravação e reprodução
static uint slice;                         // slice de PWM usado

/* ==== Buffer para o "osciloscópio" ==== */
static uint8_t  scope_buf[OLED_W];         // 1 valor por coluna (0-31)
static uint8_t  col_write = 0;             // coluna que será escrita agora
static uint8_t  col_draw  = 0;             // próxima coluna a desenhar no display

/* ==== Controle dos LEDs ==== */
static inline void led_rgb(bool r, bool g) {
    gpio_put(LED_R_PIN, r);
    gpio_put(LED_G_PIN, g);
}
#define led_idle() led_rgb(0, 0)   // LED apagado
#define led_rec()  led_rgb(1, 0)   // LED vermelho
#define led_play() led_rgb(0, 1)   // LED verde

/* ==== Funções de texto no OLED ==== */
static void oled_setup(void){
    i2c_init(i2c1, 400000);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_pull_up(14); gpio_pull_up(15);
    disp.external_vcc = false;
    ssd1306_init(&disp, OLED_W, OLED_H, 0x3C, i2c1);
}

static void oled_center(const char *s, int y){
    int x = (OLED_W - (int)strlen(s)*6) / 2;
    ssd1306_draw_string(&disp, x, y, 1, s);
}

static void oled_idle(void){
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
    oled_center("Audio gravado", 20);
    oled_center("Botao B -> reproduzir", 32);
    ssd1306_show(&disp);
}

/* ==== Funções auxiliares ==== */
static inline bool pressed(uint g){ return gpio_get(g) == 0; }      // botão pressionado?
static inline uint16_t abs16(int32_t v){ return v < 0 ? -v : v; }   // valor absoluto

/* ==== Desenho de uma coluna do escopo (formato da onda) ==== */
static void draw_scope_col(uint8_t x, uint8_t mag){ // mag varia de 0 a 31
    for(int y = 0; y < OLED_H; y++) 
        ssd1306_clear_pixel(&disp, x, y);

    for(int dy = 0; dy <= mag; dy++){
        ssd1306_draw_pixel(&disp, x, OLED_H / 2 + dy);  // parte de baixo
        ssd1306_draw_pixel(&disp, x, OLED_H / 2 - dy);  // parte de cima
    }
}

/* ==== Atualiza o OLED com o conteúdo do scope_buf ==== */
static void scope_refresh(void){
    if(st != PLAY) return;

    while(col_draw != col_write){
        uint8_t h = scope_buf[col_draw];
        draw_scope_col(col_draw, h);
        col_draw = (col_draw + 1) & 0x7F;  // (mod 128)
    }
    ssd1306_show(&disp);
}

/* ==== Callback de gravação ==== */
static bool rec_cb(struct repeating_timer *t){
    if(rec_i >= N_SAMPLES) return false;
    buf[rec_i++] = adc_read();    // salva a amostra
    return true;
}

/* ==== Callback de reprodução ==== */
static bool play_cb(struct repeating_timer *t){
    if(play_i >= N_SAMPLES) return false;

    // PWM: aplica suavização no valor de saída
    int32_t diff = (int32_t)buf[play_i++] - 2048;
    int32_t x = (diff * 12) / 2 + 512;
    static int32_t y = 512;
    y += (x - y) >> 2;
    if(y < 0) y = 0; 
    else if(y > 1023) y = 1023;
    pwm_set_gpio_level(PWM_PIN, (uint16_t)y);

    // Scope: salva valor absoluto normalizado (0-31)
    uint16_t a = abs16(diff);
    if(a > scope_buf[col_write])
        scope_buf[col_write] = a >> 6;
    if(++col_write == OLED_W) col_write = 0;

    return true;
}

/* ==== Inicialização do hardware ==== */
static void init_hw(void){
    stdio_init_all();
    oled_setup(); oled_idle();

    adc_init(); adc_gpio_init(MIC_PIN); adc_select_input(MIC_CH);

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

/* ==== Loop principal ==== */
int main(void){
    init_hw(); sleep_ms(400);     // espera OLED estabilizar
    struct repeating_timer tim;

    while(1){
        /* ----- Começar Gravação ----- */
        if(st == IDLE && pressed(BTN_A)){
            rec_i = 0;
            add_repeating_timer_us(-(int64_t)US_STEP, rec_cb, NULL, &tim);
            st = REC; oled_recording(); led_rec();
            while(pressed(BTN_A)) tight_loop_contents();  // espera soltar botão
        }

        /* ----- Começar Reprodução ----- */
        if(st == IDLE && pressed(BTN_B) && rec_i == N_SAMPLES){
            play_i = 0; col_write = col_draw = 0;
            memset(scope_buf, 0, sizeof scope_buf);
            ssd1306_clear(&disp); ssd1306_show(&disp);
            pwm_set_enabled(slice, true);
            add_repeating_timer_us(-(int64_t)US_STEP, play_cb, NULL, &tim);
            st = PLAY; led_play();
            while(pressed(BTN_B)) tight_loop_contents();  // espera soltar botão
        }

        /* ----- Fim da Gravação ----- */
        if(st == REC && rec_i >= N_SAMPLES){
            st = IDLE; oled_recorded(); led_idle();
        }

        /* ----- Fim da Reprodução ----- */
        if(st == PLAY && play_i >= N_SAMPLES){
            pwm_set_enabled(slice, false);
            st = IDLE; oled_idle(); led_idle();
        }

        scope_refresh();  // atualiza tela (forma de onda)
        sleep_ms(4);      // pequena pausa
    }
}
