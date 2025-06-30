# FURA-BALÃO – ESP32 Robot Firmware
Controle web + Wi-Fi Direct para mini-robôs de duelo “fura balão”

[![Arduino IDE](https://img.shields.io/static/v1?label=IDE&message=Arduino%201.8%2B&color=informational)](https://arduino.cc)
[![ESP32 core](https://img.shields.io/static/v1?label=ESP32%20core&message=%3E%3D2.0.16&color=success)](https://github.com/espressif/arduino-esp32)
[![License](https://img.shields.io/github/license/jardelgodinho/Fura_Balao-Mecatronica_IFSC)](#licença)

---

## 1 ▪ Sobre o projeto
Este firmware transforma um **ESP32** em ponto-de-acesso Wi-Fi (sem internet) e servidor web.  
Qualquer smartphone se conecta escaneando um QR code, abre `http://192.168.4.1` e controla o robô por um **joystick virtual** + três botões (LED, arma, auxiliar) – **sem app** nem loja de aplicativos.

O código foi criado para a disciplina *Introdução a Engenharia da 1ª fase de Engenharia Mecatrônica do IFSC - Campus Florianópolis (2025.1)* e serve como referência para à competição de robôs “fura balão”, onde cada robô usa um palito para estourar o balão do oponente.

---

## 2 ▪ Hardware
O firmware foi desenvolvido para utilização de um ESP32 que conecta-se com um Driver de Motor com ponte H do modelo L298. Apesar do firmware permitir o controle de um servomotor para automação da arma e monitoramento da bateria, não foi testado tão pouco trás detalhes além dos pinos de conexão para estes periféricos.

| Função                       | ESP32 | L298N |
|------------------------------|:----:|--------------------|
| PWM – motor esquerdo         | **14** | ENA |
| Dir A – motor esquerdo       | **27** | IN1 |
| Dir B – motor esquerdo       | **26** | IN2 |
| PWM – motor direito          | **32** | ENB |
| Dir A – motor direito        | **25** | IN3 |
| Dir B – motor direito        | **33** | IN4 |
| Arma / servo                 | **12** | driver/transistor |
| Leitura da bateria (VBAT)    | **34** | divisor 2:1 → +BAT |
| LED on-board                 | **2**  | diagnóstico |

> **Alimentação**  
> • Conecte o pack de baterias à entrada **12 V** do L298.  
> • Use **o pino 5 V de saída** do L298 para alimentar o pino 5 V do ESP32 (GND comum).  
> • Instale uma chave geral **entre o pack de baterias e o L298**.

---

## 3 ▪ Dependências (Arduino IDE)

1. **Placa ESP32**  
   Ferramentas ▸ Placa ▸ Gerenciador de Placas → instale **“esp32”** (≥ 2.0.16).  
2. **Bibliotecas**  
   Sketch ▸ Incluir Biblioteca ▸ Gerenciar Bibliotecas  
   * `AsyncTCP` (fork esphome)  
   * `ESPAsyncWebServer`  
   * `ArduinoJson` (≥ 6.21)  

⚠️ Núcleos 3.x exigem o *patch* de compatibilidade MBEDTLS já incluso no código.
⚠️ Requer as bibliotecas AsyncTCP  e  ESPAsyncWebServer indisponíveis na IDE Arduino, recomendo baixar em:
 *    https://github.com/esphome/AsyncTCP
 *    https://github.com/esphome/ESPAsyncWebServer 
---

## 4 ▪ Compilação & upload

Placa : ESP32 Dev Module (ou equivalente)
Flash mode : QIO
Flash freq : 80 MHz
Flash size : 4 MB
Part. scheme : Default 4MB with spiffs
Monitor baud : 115200 - Verifique este parâmetro se não estiver recebendo dados válidos no monitor serial.


1. Abra `Fura_Balao.ino` no Arduino IDE.  
2. **Altere** `AP_SSID` e `AP_PASS` para o nome da sua equipe ← provavelmente nas linas 79 e 80.
3. Conecte o ESP32, selecione a porta correta e clique **Upload**.  
4. Após `==== SETUP OK ====`, o ponto-de-acesso já está ativo.

---

## 5 ▪ Wi-Fi QR code

Gere um QR code do tipo **Wi-Fi** (WPA/WPA2) em serviços como [qifi.org](https://qifi.org) ou [qr-code-generator.com](https://www.qr-code-generator.com):

SSID : FURA-BALAO ← seu AP_SSID
Password : 12345678 ← seu AP_PASS
Security : WPA/WPA2


Imprima e cole no robô.

---

## 6 ▪ Como operar

1. **Escaneie** o QR code com o smartphone e aceite a conexão sem internet.  
2. Abra o **Chrome** (ou outro navegador moderno) e digite `192.168.4.1`.  
3. Pressione o botão **LIGAR** por **3 seg** → texto muda para **LIGADO**.  
4. Use o **joystick circular** para mover (→ 0°, ↑ 90°, ↓ 180°, ← 270°).  
5. Ative **LED**, **ARMA** ou **AUX** pelos botões coloridos ← Se disponíveis no robô. 
6. Terminou? Desligue o robô, **esqueça** a rede no telefone e entregue ao próximo participante.

---

## 7 ▪ Personalização rápida

| Parâmetro | Onde mudar | Efeito |
|-----------|-----------|--------|
| `leftTrim`, `rightTrim` | seção *Calibração* | Compensa velocidade entre motores diferentes |
| `maxPower` | seção *Calibração* | Limita potência global |
| `PWM_F`, `PWM_B` | seção *PINOUT & PWM* | Freq./resolução do PWM |
| `RAMP_MS`, `KICK_MS` | seção *Calibração* | Rampa e “kick-start” |
| Watchdog (5 s) | `loop()` | Segurança sem comando |

---

## 8 ▪ Troubleshooting

| Sintoma | Possível causa / solução |
|---------|-------------------------|
| ESP32 não faz boot | Pino 12 alto no reset → mantenha LOW ou mude de pino |
| SSID não aparece | Verifique `AP_SSID`, reinicie o robô |
| Wi-Fi cai ao acelerar | Tensão insuficiente → confira fiação 5 V/GND e bateria |
| Direções invertidas | Ajuste as conexões dos motores com o L298 (lado e/ou inverta a polaridade) |

---

## 9 ▪ Contribuição
Pull requests são bem-vindos!  
Mantenha comentários em **português**, código e variáveis em **inglês** e siga o estilo do projeto.

---

## 10 ▪ Licença
Distribuído sob a licença **MIT**. Veja o arquivo `LICENSE` para detalhes.

---

Divirta-se!
