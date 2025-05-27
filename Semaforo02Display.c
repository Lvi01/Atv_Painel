// -----------------------------------------------------------------------------
// Projeto: Painel de Controle Interativo com Acesso Concorrente
// Autor: Levi Silva Freitas
// -----------------------------------------------------------------------------

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "pico/bootrom.h"

#include "lib/ssd1306.h"

#include "pio_matrix.pio.h"

// ------------------------------ Definições -----------------------------------
// Configurações do display OLED
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define OLED_ADDR 0x3C

// Configurações dos botões
#define BTN_ENTRADA 5
#define BTN_SAIDA   6
#define BTN_RESET   22

// Configurações do led RRG
#define LED_VERDE    11
#define LED_VERMELHO 13
#define LED_AZUL     12

// Configurações do buzzer
#define BUZZER      21
#define BUZZER_PWM_SLICE    pwm_gpio_to_slice_num(BUZZER)
#define BUZZER_PWM_CHANNEL  pwm_gpio_to_channel(BUZZER)

// Define o número máximo de usuários ativos
#define MAX_USUARIOS 8

// Define o número de pixels usados para o desenho
#define NUM_PIXELS 25

// Vetor de desenho para os números 0, 1, 2, 3 e 4.
double Numeros[225] = {
    //Numero 0
    0.0, 1.0, 1.0, 1.0, 0.0,
    1.0, 1.0, 0.0, 0.0, 1.0,
    1.0, 0.0, 1.0, 0.0, 1.0,
    1.0, 0.0, 0.0, 1.0, 1.0,
    0.0, 1.0, 1.0, 1.0, 0.0,

    // Numero 1
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 0.0, 
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0,

    //Numero 2
    0.0, 0.0, 1.0, 1.0, 0.0,
    1.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 1.0,

    //Numero 3
    0.0, 1.0, 1.0, 1.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0,

    //Numero 4
    1.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 1.0, 
    1.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,

    //Numero 5
    1.0, 1.0, 1.0, 1.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 1.0, 1.0, 1.0, 1.0,

    // Numero 6
    1.0, 1.0, 1.0, 1.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 0.0, 0.0, 0.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0,

    //Numero 7
    0.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0,

    //Numero 8
    0.0, 1.0, 1.0, 1.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 1.0, 1.0, 1.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 1.0, 1.0, 1.0, 0.0,
    };

// -------------------------- Variáveis Globais -------------------------------
ssd1306_t display;
SemaphoreHandle_t semContagem;
SemaphoreHandle_t semReset;
SemaphoreHandle_t mutexDisplay;

volatile uint8_t usuariosAtivos = 0;

// Variável para debounce do botão de reset
static uint64_t debounce_antes = 0;

// -------------------------- Prototipação -------------------------------------
void taskEntrada(void *params);
void taskSaida(void *params);
void taskReset(void *params);
void taskMatrizNumeros(void *params); // Nova task para matriz de LEDs

void atualizarDisplay();
void atualizarLedRGB();
void beepCurto();
void beepDuplo();
void configurarPWM(uint gpio);
void gpioIRQHandler(uint gpio, uint32_t events);
void inicializarPerifericos();
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b);
uint32_t matrix_rgb(double b, double r, double g);

// ---------------------------- Função Main ----------------------------------
int main() {
    stdio_init_all();

    inicializarPerifericos();

    // Cria semáforos e mutex
    semContagem = xSemaphoreCreateCounting(MAX_USUARIOS, MAX_USUARIOS);
    semReset = xSemaphoreCreateBinary();
    mutexDisplay = xSemaphoreCreateMutex();

    // Inicializa o display OLED e o LED RGB
    atualizarLedRGB();
    atualizarDisplay();

    // Funções auxiliares para desenho no PIO
    PIO pio = pio0; // PIO 0
    void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b);
    uint32_t matrix_rgb(double b, double r, double g);

    // Cria tarefas
    xTaskCreate(taskEntrada, "Entrada", 256, NULL, 1, NULL);
    xTaskCreate(taskSaida, "Saida", 256, NULL, 1, NULL);
    xTaskCreate(taskReset, "Reset", 256, NULL, 1, NULL);
    xTaskCreate(taskMatrizNumeros, "MatrizNum", 512, NULL, 1, NULL); // Nova task

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
        bool cor = true;
        ssd1306_fill(&display, false);

        ssd1306_rect(&display, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
        ssd1306_line(&display, 3, 25, 123, 25, cor);           // Desenha uma linha
        ssd1306_line(&display, 3, 37, 123, 37, cor);           // Desenha uma linha

        ssd1306_draw_string(&display, "CEPEDI   TIC37", 8, 6); // Desenha uma string
        ssd1306_draw_string(&display, "EMBARCATECH", 20, 16);  // Desenha uma string
        ssd1306_draw_string(&display, "    Painel", 10, 28);   // Desenha uma string

        snprintf(buffer, sizeof(buffer), "Usuarios  %d/%d", usuariosAtivos, MAX_USUARIOS);
        ssd1306_draw_string(&display, buffer, 12, 45);
        ssd1306_line(&display, 85, 38, 85, 60, cor);           // Linha vertical
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
    uint64_t agora = to_us_since_boot(get_absolute_time());
    if ((agora - debounce_antes) < 200000)
        return;
    debounce_antes = agora;
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
            // Esvazia o semáforo de contagem
            while (uxSemaphoreGetCount(semContagem) > 0) {
                xSemaphoreTake(semContagem, 0);
            }
            // Preenche o semáforo até o valor máximo
            for (int i = 0; i < MAX_USUARIOS; i++) {
                xSemaphoreGive(semContagem);
            }
            usuariosAtivos = 0;
            atualizarLedRGB();
            atualizarDisplay();
            beepDuplo();
        }
    }
}

// -------------------- Task para Matriz de LEDs 5x5 --------------------
// Exibe o número de usuários ativos na matriz de LEDs 5x5 SOMENTE quando houver alteração
void taskMatrizNumeros(void *params) {
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &pio_matrix_program);
    pio_matrix_program_init(pio, sm, offset, 7); // Ajuste o pino conforme seu hardware

    uint8_t ultimo_numero = 255; // Valor impossível para garantir atualização na primeira vez

    while (true) {
        uint8_t numero = usuariosAtivos;
        if (numero > 8) numero = 8;

        // Só atualiza se o número mudou
        if (numero != ultimo_numero) {
            ultimo_numero = numero;
            double *desenho = &Numeros[numero * NUM_PIXELS];

            // Define as cores conforme o padrão do LED RGB
            double r = 0.0, g = 0.0, b = 0.0;
            if (numero == 0) {
                b = 0.2; // Azul fraco
            } else if (numero < MAX_USUARIOS - 1) {
                g = 0.2; // Verde fraco
            } else if (numero == MAX_USUARIOS - 1) {
                r = 0.2; g = 0.2; // Amarelo fraco
            } else {
                r = 0.2; // Vermelho fraco
            }

            // Gira a matriz 180°: percorre de trás para frente
            for (int i = NUM_PIXELS - 1; i >= 0; i--) {
                double brilho = desenho[i] ? 1.0 : 0.0; // 1.0 = brilho total, 0.0 = apagado
                uint32_t cor_led = matrix_rgb(b * brilho, r * brilho, g * brilho);
                pio_sm_put_blocking(pio, sm, cor_led);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Checa frequentemente, mas só atualiza se mudar
    }
}

uint32_t matrix_rgb(double b, double r, double g)
{
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// Função para fazer os desenhos 
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b) {
    for (int letra = 0; letra < 5; letra++) { // 5 letras, cada uma com 25 LEDs
        for (int16_t i = 0; i < NUM_PIXELS; i++) {
            // Calcular o índice correto na matriz para cada letra
            int indice = (letra * 25) + (24 - i); 
            valor_led = matrix_rgb(desenho[indice], r, g);
            pio_sm_put_blocking(pio, sm, valor_led);
        }
        sleep_ms(1500); // Intervalo de 1,5 segundo antes de acender a próxima letra
    }
}