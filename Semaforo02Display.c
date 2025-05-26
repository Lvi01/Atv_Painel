// -----------------------------------------------------------------------------
// Projeto: Painel de Controle Interativo com Acesso Concorrente
// Autor: [Seu Nome Aqui]
// Descrição: Sistema de controle de acessos simultâneos usando FreeRTOS
// Recursos: OLED, LED RGB, Buzzer, Semáforos, Mutex, Interrupções
// Placa: BitDogLab (RP2040)
// -----------------------------------------------------------------------------

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pico/bootrom.h"
#include "lib/ssd1306.h"

// ------------------------------ Definições -----------------------------------
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define OLED_ADDR 0x3C

#define BTN_ENTRADA 5
#define BTN_SAIDA   6
#define BTN_RESET   7

#define LED_VERDE    11
#define LED_VERMELHO 13
#define LED_AZUL     12

#define BUZZER      21

#define BUZZER_PWM_SLICE    pwm_gpio_to_slice_num(BUZZER)
#define BUZZER_PWM_CHANNEL  pwm_gpio_to_channel(BUZZER)

#define MAX_USUARIOS 8

// -------------------------- Variáveis Globais -------------------------------
ssd1306_t display;
SemaphoreHandle_t semContagem;
SemaphoreHandle_t semReset;
SemaphoreHandle_t mutexDisplay;

volatile uint8_t usuariosAtivos = 0;

// -------------------------- Prototipação -------------------------------------
void taskEntrada(void *params);
void taskSaida(void *params);
void taskReset(void *params);

void atualizarDisplay();
void atualizarLedRGB();
void beepCurto();
void beepDuplo();
void configurarPWM(uint gpio);
void gpioIRQHandler(uint gpio, uint32_t events);
void inicializarPerifericos();

// ---------------------------- Função Main ----------------------------------
int main() {
    stdio_init_all();

    inicializarPerifericos();

    // Cria semáforos e mutex
    semContagem = xSemaphoreCreateCounting(MAX_USUARIOS, 0); // Semáforo para controle de contagem de usuários
    semReset = xSemaphoreCreateBinary();                    // Semáforo para resetar o contador
    mutexDisplay = xSemaphoreCreateMutex();                // Mutex para proteger o acesso ao display OLED

    // Cria tarefas
    xTaskCreate(taskEntrada, "Entrada", 256, NULL, 1, NULL); // Task para entrada de usuários
    xTaskCreate(taskSaida, "Saida", 256, NULL, 1, NULL);    // Task para saída de usuários
    xTaskCreate(taskReset, "Reset", 256, NULL, 1, NULL);   // Task para resetar o contador

    // Inicia o agendador
    vTaskStartScheduler();
    while (1) {}
    return 0;
}

// -------------------- Inicialização --------------------
// Inicializa os periféricos: I2C, Display OLED, Botões, LEDs RGB e Buzzer
void inicializarPerifericos() {
    // I2C e Display
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&display, WIDTH, HEIGHT, false, OLED_ADDR, I2C_PORT);
    ssd1306_config(&display);
    ssd1306_fill(&display, false);
    ssd1306_send_data(&display);

    // Botões
    gpio_init(BTN_ENTRADA);
    gpio_set_dir(BTN_ENTRADA, GPIO_IN);
    gpio_pull_up(BTN_ENTRADA);

    gpio_init(BTN_SAIDA);
    gpio_set_dir(BTN_SAIDA, GPIO_IN);
    gpio_pull_up(BTN_SAIDA);

    gpio_init(BTN_RESET);
    gpio_set_dir(BTN_RESET, GPIO_IN);
    gpio_pull_up(BTN_RESET);
    gpio_set_irq_enabled_with_callback(BTN_RESET, GPIO_IRQ_EDGE_FALL, true, gpioIRQHandler);

    // LEDs RGB e Buzzer
    configurarPWM(LED_VERMELHO);
    configurarPWM(LED_VERDE);
    configurarPWM(LED_AZUL);
    configurarPWM(BUZZER);

    // Configura PWM do Buzzer
    pwm_set_wrap(BUZZER_PWM_SLICE, 12500);
    pwm_set_clkdiv(BUZZER_PWM_SLICE, 125.0f);
    pwm_set_enabled(BUZZER_PWM_SLICE, true);
}

void configurarPWM(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}

// -------------------- Funções Auxiliares --------------------
// Atualiza o LED RGB com base no número de usuários ativos
void atualizarLedRGB() {
    if (usuariosAtivos == 0) {
        pwm_set_gpio_level(LED_VERMELHO, 0);
        pwm_set_gpio_level(LED_VERDE, 0);
        pwm_set_gpio_level(LED_AZUL, 255);
    } else if (usuariosAtivos < MAX_USUARIOS - 1) {
        pwm_set_gpio_level(LED_VERMELHO, 0);
        pwm_set_gpio_level(LED_VERDE, 255);
        pwm_set_gpio_level(LED_AZUL, 0);
    } else if (usuariosAtivos == MAX_USUARIOS - 1) {
        pwm_set_gpio_level(LED_VERMELHO, 255);
        pwm_set_gpio_level(LED_VERDE, 255);
        pwm_set_gpio_level(LED_AZUL, 0);
    } else {
        pwm_set_gpio_level(LED_VERMELHO, 255);
        pwm_set_gpio_level(LED_VERDE, 0);
        pwm_set_gpio_level(LED_AZUL, 0);
    }
}

// Atualiza o display OLED com o número de usuários ativos
void atualizarDisplay() {
    if (xSemaphoreTake(mutexDisplay, portMAX_DELAY) == pdTRUE) {
        char buffer[32];
        ssd1306_fill(&display, false);
        snprintf(buffer, sizeof(buffer), "Usuarios: %d/%d", usuariosAtivos, MAX_USUARIOS);
        ssd1306_draw_string(&display, buffer, 5, 25);
        ssd1306_send_data(&display);
        xSemaphoreGive(mutexDisplay);
    }
}

// -------------------- Beeps --------------------
// Funções para emitir beeps curtos e duplos usando PWM
void beepCurto() {
    pwm_set_chan_level(BUZZER_PWM_SLICE, BUZZER_PWM_CHANNEL, 6250);
    vTaskDelay(pdMS_TO_TICKS(200));
    pwm_set_chan_level(BUZZER_PWM_SLICE, BUZZER_PWM_CHANNEL, 0);
}

void beepDuplo() {
    for (int i = 0; i < 2; i++) {
        pwm_set_chan_level(BUZZER_PWM_SLICE, BUZZER_PWM_CHANNEL, 6250);
        vTaskDelay(pdMS_TO_TICKS(150));
        pwm_set_chan_level(BUZZER_PWM_SLICE, BUZZER_PWM_CHANNEL, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

// -------------------- Interrupção do Botão de Reset --------------------
// Esta função é chamada quando o botão de reset é pressionado.
// Ela libera o semáforo de reset, que é usado pela taskReset para zerar o contador.
void gpioIRQHandler(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(semReset, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// -------------------- Tasks --------------------
// Task de Entrada: Incrementa o contador de usuários ativos
// e atualiza o display e LEDs RGB.
// Se o semáforo estiver cheio, emite um beep curto.
void taskEntrada(void *params) {
    while (true) {
        if (!gpio_get(BTN_ENTRADA)) {
            if (xSemaphoreTake(semContagem, 0) == pdTRUE) {
                usuariosAtivos++;
                atualizarLedRGB();
                atualizarDisplay();
            } else {
                beepCurto();
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task de Saída: Decrementa o contador de usuários ativos
// e atualiza o display e LEDs RGB.
// Se não houver usuários ativos, não faz nada.
void taskSaida(void *params) {
    while (true) {
        if (!gpio_get(BTN_SAIDA)) {
            if (usuariosAtivos > 0) {
                usuariosAtivos--;
                xSemaphoreGive(semContagem);
                atualizarLedRGB();
                atualizarDisplay();
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task de Reset: Reseta o contador de usuários ativos
// e atualiza o display e LEDs RGB.
// Emite um beep duplo após o reset.
void taskReset(void *params) {
    while (true) {
        if (xSemaphoreTake(semReset, portMAX_DELAY) == pdTRUE) {
            while (usuariosAtivos < MAX_USUARIOS) {
                xSemaphoreGive(semContagem);
                usuariosAtivos++;
            }
            usuariosAtivos = 0;
            atualizarLedRGB();
            atualizarDisplay();
            beepDuplo();
        }
    }
}