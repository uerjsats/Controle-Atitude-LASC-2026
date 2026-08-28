#include <Arduino.h>

// =====================================================
// PINOS DOS MOTORES (XIAO ESP32-S3)
// =====================================================
#define MOTOR1 1
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 9
#define MOTOR5 7

// =====================================================
// CONFIGURAÇÃO PWM
// =====================================================
#define PWM_FREQ        20000
#define PWM_RESOLUTION  8       // 0 a 255

// =====================================================
// COMUNICAÇÃO UART (CRTP COM OUTRO ESP32)
// =====================================================
#define UART_RX_PIN     44      // Conecte ao TX do outro ESP32
#define UART_TX_PIN     43      // Conecte ao RX do outro ESP32
#define UART_BAUDRATE   115200

#define CRTP_START_BYTE   0xAA
#define COMMANDER_HEADER  0x30

// Variáveis de controle de voo
float g_roll = 0.0, g_pitch = 0.0, g_yaw = 0.0;
uint16_t g_thrust = 0;
unsigned long last_packet_time = 0;

// =====================================================
// DECLARAÇÃO DE FUNÇÕES
// =====================================================
void acionarMotor(int motor, int potencia_percent);
void pararTodos();
void setMotorPWM(int motor_pin, int pwm_val);
void aplicarComandosVoo(float roll, float pitch, float yaw, uint16_t thrust);
void enviarReadyToFly();
void processarUART_CRTP();
void processarComandosSerialUSB();

// =====================================================
// SETUP
// =====================================================
void setup() {
  // Serial USB para depuração/comandos manuais
  Serial.begin(115200);
  delay(1000);

  // Configura PWM nos pinos dos motores
  ledcAttach(MOTOR1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR4, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR5, PWM_FREQ, PWM_RESOLUTION);

  pararTodos();

  // Inicializa UART de comunicação com o transmissor CRTP
  Serial1.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.println("========================================");
  Serial.println("  RECEPTOR CRTP + CONTROLE 5 MOTORES");
  Serial.println("           XIAO ESP32-S3");
  Serial.println("========================================");

  // Envia o pacote para liberar a transmissão do outro microcontrolador
  enviarReadyToFly();
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================
void loop() {
  // 1. Processa pacotes de voo recebidos pela UART (Serial1)
  processarUART_CRTP();

  // 2. Processa comandos manuais digitados no Monitor Serial (Serial USB)
  processarComandosSerialUSB();

  // 3. Failsafe: se ficar mais de 500ms sem receber pacote e o motor estava ligado, desliga
  if (last_packet_time > 0 && (millis() - last_packet_time > 500)) {
    if (g_thrust > 0) {
      Serial.println("[FAILSAFE] Perda de sinal CRTP! Desligando motores...");
      pararTodos();
      g_thrust = 0;
    }
  }
}

// =====================================================
// ENVIA PACOTE "READY TO FLY" (0xAA 0xAA 0xF0 0x01 0x01 0xF2)
// =====================================================
void enviarReadyToFly() {
  uint8_t rtf_packet[6] = {0xAA, 0xAA, 0xF0, 0x01, 0x01, 0xF2};
  Serial1.write(rtf_packet, sizeof(rtf_packet));
  Serial.println(">> Sinal [Ready to Fly] enviado via Serial1.");
}

// =====================================================
// PARSER CRTP (RECEBE ROLL, PITCH, YAW, THRUST)
// =====================================================
void processarUART_CRTP() {
  static uint8_t rx_state = 0;
  static uint8_t payload[14];
  static uint8_t idx = 0;
  static uint8_t checksum = 0;

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    switch (rx_state) {
      case 0: // Primeiro Start Byte
        if (c == CRTP_START_BYTE) rx_state = 1;
        break;

      case 1: // Segundo Start Byte
        if (c == CRTP_START_BYTE) rx_state = 2;
        else rx_state = 0;
        break;

      case 2: // Header do Commander
        if (c == COMMANDER_HEADER) {
          checksum = c;
          rx_state = 3;
        } else {
          rx_state = 0;
        }
        break;

      case 3: // Tamanho do payload (14 bytes)
        if (c == 14) {
          checksum += c;
          idx = 0;
          rx_state = 4;
        } else {
          rx_state = 0;
        }
        break;

      case 4: // Leitura dos 14 bytes de dados
        payload[idx++] = c;
        checksum += c;
        if (idx == 14) rx_state = 5;
        break;

      case 5: // Validação do Checksum
        if (c == checksum) {
          memcpy(&g_roll, &payload[0], 4);
          memcpy(&g_pitch, &payload[4], 4);
          memcpy(&g_yaw, &payload[8], 4);
          memcpy(&g_thrust, &payload[12], 2);

          last_packet_time = millis();

          // Aplica potências aos motores
          aplicarComandosVoo(g_roll, g_pitch, g_yaw, g_thrust);
        }
        rx_state = 0;
        break;
    }
  }
}

// =====================================================
// CONVERSÃO E DISTRIBUIÇÃO DE FORÇA NOS 5 MOTORES
// =====================================================
void aplicarComandosVoo(float roll, float pitch, float yaw, uint16_t thrust) {
  // Converte o thrust (0-65535) para escala PWM 8-bit (0-255)
  int32_t pwm_base = map(thrust, 0, 65535, 0, 255);

  if (pwm_base <= 5) {
    pararTodos();
    return;
  }

  // Motor Central (M5) assume o empuxo principal
  setMotorPWM(MOTOR5, pwm_base);

  // Motores periféricos (M1 a M4) para atitude em cruz/X
  // Usa int32_t para evitar underflow matemático antes da saturação
  int32_t sub_base = pwm_base / 2;
  int32_t m1 = sub_base + (int32_t)pitch + (int32_t)yaw;
  int32_t m2 = sub_base - (int32_t)roll  - (int32_t)yaw;
  int32_t m3 = sub_base - (int32_t)pitch + (int32_t)yaw;
  int32_t m4 = sub_base + (int32_t)roll  - (int32_t)yaw;

  setMotorPWM(MOTOR1, m1);
  setMotorPWM(MOTOR2, m2);
  setMotorPWM(MOTOR3, m3);
  setMotorPWM(MOTOR4, m4);
}

// =====================================================
// FUNÇÕES DE ACIONAMENTO COM LIMITAÇÃO (CLIPPING)
// =====================================================
void setMotorPWM(int motor_pin, int pwm_val) {
  // Garante que o PWM permaneça estritamente entre 0 e 255
  int clamped = constrain(pwm_val, 0, 255);
  ledcWrite(motor_pin, clamped);
}

void acionarMotor(int motor, int potencia_percent) {
  int pwm = map(potencia_percent, 0, 100, 0, 255);
  switch (motor) {
    case 1: setMotorPWM(MOTOR1, pwm); break;
    case 2: setMotorPWM(MOTOR2, pwm); break;
    case 3: setMotorPWM(MOTOR3, pwm); break;
    case 4: setMotorPWM(MOTOR4, pwm); break;
    case 5: setMotorPWM(MOTOR5, pwm); break;
  }
}

void pararTodos() {
  ledcWrite(MOTOR1, 0);
  ledcWrite(MOTOR2, 0);
  ledcWrite(MOTOR3, 0);
  ledcWrite(MOTOR4, 0);
  ledcWrite(MOTOR5, 0);
}

// =====================================================
// PARSER MANUAL VIA USB SERIAL
// =====================================================
void processarComandosSerialUSB() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.length() == 0) return;

    if (comando == "0") {
      pararTodos();
      Serial.println("TODOS OS MOTORES DESLIGADOS");
      return;
    }

    if (comando.equalsIgnoreCase("rtf")) {
      enviarReadyToFly();
      return;
    }

    int espaco = comando.indexOf(' ');
    if (espaco != -1) {
      int motor = comando.substring(0, espaco).toInt();
      int potencia = comando.substring(espaco + 1).toInt();

      if (motor >= 1 && motor <= 5 && potencia >= 0 && potencia <= 100) {
        acionarMotor(motor, potencia);
        Serial.printf("Motor %d -> %d%% (PWM: %d)\n", motor, potencia, map(potencia, 0, 100, 0, 255));
      } else {
        Serial.println("ERRO: Motor (1-5) ou Potencia (0-100%) invalidos.");
      }
    }
  }
}