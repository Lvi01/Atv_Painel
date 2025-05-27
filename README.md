# Painel de Controle Interativo com Acesso Concorrente 🚦🔒

**Autor:** Levi Silva Freitas  
---

## 🎯 Objetivo

Este projeto visa implementar um **painel de controle interativo** que simula o controle de entrada de usuários em um espaço físico (ex: laboratório, biblioteca). A lógica de controle utiliza **semáforos, mutex e interrupções** no FreeRTOS para garantir concorrência segura entre tarefas.

---

## ⚙️ Funcionalidades

- 👥 Controle de **entradas e saídas de usuários** via botões físicos
- 🎛️ **Limite máximo configurável** de ocupação (ex: 8 usuários)
- 🔒 Proteção de recursos compartilhados com **mutex**
- 🔁 **Semáforo de contagem** para limitar acessos simultâneos
- ⛔ **Reset do sistema** via botão com interrupção
- 🖥️ Interface visual com **Display OLED**
- 🔴🟢🔵 **Feedback com LED RGB** baseado na ocupação:
  - **Azul:** 0 usuários
  - **Verde:** usuários ativos
  - **Amarelo:** uma vaga restante
  - **Vermelho:** capacidade máxima
- 🔊 Feedback sonoro com **buzzer PWM**:
  - Beep curto: entrada negada
  - Beep duplo: reset efetuado
- 🔢 **Matriz de LEDs 5x5 (via PIO)** mostrando número atual de usuários

---

## 📦 Componentes Utilizados

| Componente        | Uso                                      |
|-------------------|-------------------------------------------|
| Raspberry Pi Pico | Microcontrolador principal                |
| Display OLED SSD1306 | Exibição de informações e estado do sistema |
| LED RGB           | Feedback visual sobre ocupação           |
| Buzzer (PWM)      | Sinalização sonora                       |
| Botões físicos    | Entrada, saída e reset                   |
| Matriz de LEDs (PIO) | Exibição numérica do contador via PIO |

---

## 🧠 Tecnologias e Conceitos

- FreeRTOS (semáforos, mutex, tarefas, interrupções)
- PWM (LED RGB e buzzer)
- I2C (comunicação com o display)
- PIO (matriz de LEDs programável)
- Concorrência segura com multitarefa

---

## 🚀 Como Usar

1. Compile o projeto com SDK do Raspberry Pi Pico e FreeRTOS.
2. Carregue o binário na placa via BOOTSEL.
3. Use os botões da BitDogLab:
   - **Botão A (GPIO 5)**: entrada de usuário
   - **Botão B (GPIO 6)**: saída de usuário
   - **Botão do Joystick (GPIO 22)**: reset
4. Acompanhe o estado no display OLED, LED RGB e som do buzzer.

---
