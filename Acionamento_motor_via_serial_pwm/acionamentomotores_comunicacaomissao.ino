#include <Arduino.h>

// =====================================================
// 1. PINOS E CONFIGURAÇÕES DE HARDWARE (XIAO ESP32-S3)
// =====================================================
#define MOTOR1             1
#define MOTOR2             3
#define MOTOR3             4
#define MOTOR4             9
#define MOTOR5             7

#define PWM_FREQ           20000
#define PWM_RESOLUTION     8

#define CRTP_TX_PIN        43       
#define CRTP_RX_PIN        44       
#define CRTP_BAUDRATE      115200   

// =====================================================
// 2. DEFINIÇÕES DO PROTOCOLO CRTP SERIAL
// =====================================================
#define CRTP_START_BYTE    0xAA     
#define CRTP_MAX_DATA_SIZE 30       

typedef struct {
    uint8_t header;
    uint8_t size;
    uint8_t data[CRTP_MAX_DATA_SIZE];
    uint8_t checksum;
} CrtpPacket_t;

typedef enum { 
    STATE_WAIT_START, 
    STATE_HEADER, 
    STATE_SIZE, 
    STATE_DATA, 
    STATE_CHECKSUM 
} CrtpRxState_t;

CrtpRxState_t rx_state = STATE_WAIT_START;
CrtpPacket_t rx_packet;
uint8_t data_idx = 0;
uint8_t calculated_checksum = 0;

// =====================================================
// 3. FUNÇÕES DE CONTROLE DOS MOTORES
// =====================================================
void acionarMotor(int motor, int potencia) {
    int pwm = map(potencia, 0, 100, 0, 255);

    switch (motor) {
        case 1: ledcWrite(MOTOR1, pwm); break;
        case 2: ledcWrite(MOTOR2, pwm); break;
        case 3: ledcWrite(MOTOR3, pwm); break;
        case 4: ledcWrite(MOTOR4, pwm); break;
        case 5: ledcWrite(MOTOR5, pwm); break;
        default: return;
    }

    Serial.printf("[MOTOR] M%d -> %d%% (PWM: %d)\n", motor, potencia, pwm);
}

void pararTodos() {
    ledcWrite(MOTOR1, 0);
    ledcWrite(MOTOR2, 0);
    ledcWrite(MOTOR3, 0);
    ledcWrite(MOTOR4, 0);
    ledcWrite(MOTOR5, 0);
}

// =====================================================
// 4. FUNÇÕES AUXILIARES E PROTOCOLO CRTP
// =====================================================
void printHexBuffer(uint8_t* buffer, uint8_t size) {
    Serial.print("DADOS (Hex): ");
    for (int i = 0; i < size; i++) {
        Serial.printf("%02X ", buffer[i]);
    }
    Serial.println(); 
}

void send_crtp_ack(uint8_t header) {
    uint8_t ack_packet[5];
    ack_packet[0] = CRTP_START_BYTE; 
    ack_packet[1] = CRTP_START_BYTE;
    ack_packet[2] = header;          
    ack_packet[3] = 0x00;
    ack_packet[4] = header + 0x00;

    Serial1.write(ack_packet, sizeof(ack_packet));
    Serial.printf("<= ACK Enviado (Porta/Canal 0x%02X)\n\n", header);
}

void handle_crtp_payload(uint8_t port, uint8_t channel, uint8_t* data, uint8_t size) {
    // Exemplo de integração: se o payload tiver [motor, potencia]
    // if (size >= 2) {
    //     acionarMotor(data[0], data[1]);
    // }
}

void process_crtp_byte(uint8_t byte) {
    switch (rx_state) {
        case STATE_WAIT_START:
            if (byte == CRTP_START_BYTE) rx_state = STATE_HEADER;
            break;

        case STATE_HEADER:
            if (byte == CRTP_START_BYTE) break;
            rx_packet.header = byte;
            calculated_checksum = byte;
            rx_state = STATE_SIZE;
            break;

        case STATE_SIZE:
            rx_packet.size = byte;
            calculated_checksum += byte;
            if (rx_packet.size > CRTP_MAX_DATA_SIZE) {
                rx_state = STATE_WAIT_START;
            } else if (rx_packet.size == 0) {
                rx_state = STATE_CHECKSUM;
            } else {
                data_idx = 0;
                rx_state = STATE_DATA;
            }
            break;

        case STATE_DATA:
            rx_packet.data[data_idx++] = byte;
            calculated_checksum += byte;
            if (data_idx >= rx_packet.size) rx_state = STATE_CHECKSUM;
            break;

        case STATE_CHECKSUM:
            rx_packet.checksum = byte;
            
            if (calculated_checksum == rx_packet.checksum) {
                uint8_t port = (rx_packet.header >> 4) & 0x0F;
                uint8_t channel = rx_packet.header & 0x0F;
                
                Serial.printf("=> RECEBIDO SERIAL1 | Porta: %d, Canal: %d, Tamanho: %d bytes\n", port, channel, rx_packet.size);
                if (rx_packet.size > 0) {
                    printHexBuffer(rx_packet.data, rx_packet.size);
                }
                
                handle_crtp_payload(port, channel, rx_packet.data, rx_packet.size);
                send_crtp_ack(rx_packet.header);
            } else {
                Serial.println("Erro: Falha de Checksum no pacote serial CRTP!");
            }
            
            rx_state = STATE_WAIT_START;
            break;
    }
}

// =====================================================
// 5. PARSER DE COMANDOS DO MONITOR SERIAL (Serial USB)
// =====================================================
void process_serial_command() {
    if (Serial.available() <= 0) return;

    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.length() == 0) return;

    if (comando == "0") {
        pararTodos();
        Serial.println("\nTODOS OS MOTORES DESLIGADOS\n");
        return;
    }

    int espaco = comando.indexOf(' ');
    if (espaco == -1) {
        Serial.println("\n[ERRO] Formato correto: MOTOR POTENCIA (ex: 2 50)\n");
        return;
    }

    int motor = comando.substring(0, espaco).toInt();
    int potencia = comando.substring(espaco + 1).toInt();

    if (motor < 1 || motor > 5) {
        Serial.println("\n[ERRO] Motor deve ser de 1 a 5.\n");
        return;
    }

    if (potencia < 0 || potencia > 100) {
        Serial.println("\n[ERRO] Potência deve estar entre 0 e 100%.\n");
        return;
    }

    acionarMotor(motor, potencia);
}

// =====================================================
// SETUP & LOOP
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    // Inicialização do PWM nos pinos dos motores
    ledcAttach(MOTOR1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR3, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR4, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR5, PWM_FREQ, PWM_RESOLUTION);
    pararTodos();

    // Inicialização da UART para o CRTP
    Serial1.begin(CRTP_BAUDRATE, SERIAL_8N1, CRTP_RX_PIN, CRTP_TX_PIN);

    Serial.println("========================================");
    Serial.println("   XIAO ESP32-S3: CRTP RX + 5 MOTORES   ");
    Serial.println("========================================");
    Serial.println("Controle via Serial USB:");
    Serial.println("  '0'       -> Desliga todos os motores");
    Serial.println("  '<M> <P>' -> Liga Motor M (1-5) na potência P% (0-100)");
    Serial.println("UART1 CRTP ativa (RX: 44, TX: 43)...");
    Serial.println("========================================\n");
}

void loop() {
    // 1. Processa comandos de texto vindos da Serial USB
    process_serial_command();

    // 2. Processa bytes binários CRTP vindos da Serial1
    while (Serial1.available() > 0) {
        uint8_t incoming_byte = Serial1.read();
        process_crtp_byte(incoming_byte);
    }
}
