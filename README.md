# Projetos de Sistemas Embarcados - EmbarcaTech 2025
**Autor:** Wagner Cutrim Rabelo Junior  
**Curso:** Residência Tecnológica em Sistemas Embarcados  
**Instituição:** EmbarcaTech - HBr  
**Brasília, 08 de Junho de 2025**

---

## 🎯 Projeto: Gravador e Reprodutor de Áudio com Visualização em Display OLED

### 🧾 Descrição Geral
Este projeto implementa um sistema embarcado que permite gravar e reproduzir áudio utilizando o microcontrolador RP2040 (BitDogLab). O sistema integra componentes como microfone analógico, saída de áudio via PWM, joystick para configuração de tempo de gravação, botões físicos para controle, LEDs RGB como indicadores visuais e um display OLED para feedback textual e visualização do áudio em tempo real.

### 🎥 Vídeo de Demonstração

https://www.youtube.com/shorts/Z2gVA09CP5w

### 🧠 Funcionalidades Principais
- **Gravação de áudio** (até 15 segundos) via ADC.
- **Reprodução de áudio** com filtro simples por PWM.
- **Ajuste dinâmico de tempo** de gravação com o joystick (1 a 15 segundos).
- **Feedback visual** com LED RGB: vermelho (gravando), verde (reproduzindo), apagado (ocioso).
- **Visualização do áudio** como osciloscópio no display OLED (durante reprodução).
- **Interface amigável no display OLED** com instruções em tempo real.

### ⚙️ Hardware Utilizado
- **Placa**: BitDogLab (RP2040)
- **Microfone**: analógico (ligado ao ADC2 / GP28)
- **Saída de Áudio**: pino PWM (GP21) conectado a um filtro passa-baixa
- **Display OLED**: 128x64 I2C (conectado aos pinos GP14/GP15)
- **Joystick**: eixo X (ADC0 / GP26)
- **Botões**: botão A (GP5), botão B (GP6)
- **LED RGB**: LED vermelho (GP13), LED verde (GP11)

### 🛠️ Bibliotecas e Dependências
- **Pico SDK 1.5+**
- **SSD1306** (driver I2C customizado incluso no projeto)
- **pico/stdlib**, **hardware/adc**, **hardware/pwm**, **hardware/i2c**

### 🔁 Ciclo de Uso
1. **Ligar o dispositivo**: Display mostra “Botão A → gravar” e segundos configuráveis.
2. **Usar o joystick para ajustar** o tempo de gravação.
3. **Pressionar o botão A** para iniciar a gravação.
4. **Gravação concluída** → exibe instruções para reprodução.
5. **Pressionar o botão B** → inicia a reprodução com visualização do áudio no OLED.
6. **Sistema retorna ao modo ocioso** após reprodução.

### 🧪 Detalhes Técnicos
- **Taxa de amostragem**: 8000 Hz
- **Resolução do ADC**: 12 bits
- **PWM configurado** com `wrap=1023` e `clkdiv=4.0`
- **Buffer de áudio**: 120000 amostras (15 s @ 8 kHz)
- **Filtro digital simples** com média exponencial (1ª ordem)
- **Visualização no display OLED** utiliza 128 colunas (1 byte por coluna)

### 📌 Observações
- O sistema evita interferência no joystick durante gravação/reprodução ao desabilitar a leitura do eixo X fora do modo ocioso.
- O escopo só é atualizado durante reprodução (modo `PLAY`).

---

## 📜 Licença
GNU GPL-3.0.

