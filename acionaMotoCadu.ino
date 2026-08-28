```cpp
// ============================================================
// XIAO ESP32-S3
// RECEPTOR CRTP + CONTROLE DE 5 MOTORES
//
// UART CRTP:
//   RX = GPIO 44
//   TX = GPIO 43
//   Baud = 115200
//
// CRTP:
//   Porta  = 13
//   Canal  = 2
//
// PAYLOAD PRINCIPAL:
//   Byte 0 -> Motor 1 PWM (0-255)
//   Byte 1 -> Motor 2 PWM (0-255)
//   Byte 2 -> Motor 3 PWM (0-255)
//   Byte 3 -> Motor 4 PWM (0-255)
//   Byte 4 -> Motor 5 PWM (0-255)
//
// FRAME:
//   0xAA
//   HEADER
//   SIZE
//   DATA...
//   CHECKSUM
//
// CHECKSUM:
//   HEADER + SIZE + todos os bytes DATA
//   limitado a uint8_t
// ============================================================

#include <Arduino.h>


// ============================================================
// 1. PINOS DOS MOTORES
// ============================================================

#define MOTOR1 1
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 9
#define MOTOR5 7


// ============================================================
// 2. UART DO CRTP
// ============================================================

#define CRTP_TX_PIN 43
#define CRTP_RX_PIN 44
#define CRTP_BAUDRATE 115200


// ============================================================
// 3. CONFIGURAÇÃO CRTP
// ============================================================

#define CRTP_START_BYTE 0xAA
#define CRTP_MAX_DATA_SIZE 30

// Porta e canal utilizados para controle dos motores
#define CRTP_MOTOR_PORT 13
#define CRTP_MOTOR_CHANNEL 2


// ============================================================
// 4. CONFIGURAÇÃO PWM
// ============================================================

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8


// ============================================================
// 5. WATCHDOG DE COMUNICAÇÃO
// ============================================================
//
// Se nenhum pacote CRTP válido chegar dentro deste tempo,
// todos os motores são desligados.
//
// 500 ms é um valor adequado para teste.
// Para voo real, esse valor deve ser definido de acordo
// com a arquitetura de controle.
// ============================================================

#define CRTP_TIMEOUT_MS 500

unsigned long ultimoPacoteValido = 0;


// ============================================================
// 6. ESTRUTURA DO PACOTE CRTP
// ============================================================

typedef struct {

    uint8_t header;

    uint8_t size;

    uint8_t data[CRTP_MAX_DATA_SIZE];

    uint8_t checksum;

} CrtpPacket_t;


// ============================================================
// 7. MÁQUINA DE ESTADOS DO RECEPTOR
// ============================================================

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


// ============================================================
// 8. ESTADO DOS MOTORES
// ============================================================

uint8_t motorPWM[5] = {

    0,
    0,
    0,
    0,
    0

};


// ============================================================
// 9. FUNÇÕES DOS MOTORES
// ============================================================


// ------------------------------------------------------------
// Configura PWM
// ------------------------------------------------------------

void configurarMotores() {

    ledcAttach(MOTOR1, PWM_FREQ, PWM_RESOLUTION);

    ledcAttach(MOTOR2, PWM_FREQ, PWM_RESOLUTION);

    ledcAttach(MOTOR3, PWM_FREQ, PWM_RESOLUTION);

    ledcAttach(MOTOR4, PWM_FREQ, PWM_RESOLUTION);

    ledcAttach(MOTOR5, PWM_FREQ, PWM_RESOLUTION);

    pararTodos();
}


// ------------------------------------------------------------
// Para todos os motores
// ------------------------------------------------------------

void pararTodos() {

    motorPWM[0] = 0;
    motorPWM[1] = 0;
    motorPWM[2] = 0;
    motorPWM[3] = 0;
    motorPWM[4] = 0;


    ledcWrite(MOTOR1, 0);

    ledcWrite(MOTOR2, 0);

    ledcWrite(MOTOR3, 0);

    ledcWrite(MOTOR4, 0);

    ledcWrite(MOTOR5, 0);
}


// ------------------------------------------------------------
// Aciona os cinco motores diretamente
// ------------------------------------------------------------

void atualizarMotores(
    uint8_t m1,
    uint8_t m2,
    uint8_t m3,
    uint8_t m4,
    uint8_t m5
) {

    motorPWM[0] = m1;
    motorPWM[1] = m2;
    motorPWM[2] = m3;
    motorPWM[3] = m4;
    motorPWM[4] = m5;


    ledcWrite(MOTOR1, m1);

    ledcWrite(MOTOR2, m2);

    ledcWrite(MOTOR3, m3);

    ledcWrite(MOTOR4, m4);

    ledcWrite(MOTOR5, m5);
}


// ============================================================
// 10. DEBUG DO PACOTE
// ============================================================

void printHexBuffer(uint8_t* buffer, uint8_t size) {

    Serial.print("DADOS: ");

    for (uint8_t i = 0; i < size; i++) {

        Serial.printf("%02X ", buffer[i]);

    }

    Serial.println();
}


// ============================================================
// 11. ACK
// ============================================================
//
// Mantém a mesma lógica do seu receptor original.
//
// Formato:
// AA AA HEADER 00 HEADER
// ============================================================

void send_crtp_ack(uint8_t header) {

    uint8_t ack_packet[5];

    ack_packet[0] = CRTP_START_BYTE;

    ack_packet[1] = CRTP_START_BYTE;

    ack_packet[2] = header;

    ack_packet[3] = 0x00;

    ack_packet[4] = header;


    Serial1.write(
        ack_packet,
        sizeof(ack_packet)
    );


    Serial.printf(
        "<= ACK enviado | Header: 0x%02X\n",
        header
    );
}


// ============================================================
// 12. PROCESSAMENTO DO PAYLOAD DOS MOTORES
// ============================================================

void processarMotorPayload(
    uint8_t* data,
    uint8_t size
) {

    // --------------------------------------------------------
    // FORMATO PRINCIPAL
    //
    // 5 bytes:
    //
    // [M1][M2][M3][M4][M5]
    //
    // Cada valor 0-255
    // --------------------------------------------------------

    if (size == 5) {

        atualizarMotores(
            data[0],
            data[1],
            data[2],
            data[3],
            data[4]
        );


        Serial.printf(
            "MOTORES -> M1:%3d M2:%3d M3:%3d M4:%3d M5:%3d\n",
            data[0],
            data[1],
            data[2],
            data[3],
            data[4]
        );

        return;
    }


    // --------------------------------------------------------
    // FORMATO ALTERNATIVO
    //
    // [0x01][M1][M2][M3][M4][M5]
    //
    // O primeiro byte funciona como comando.
    // --------------------------------------------------------

    if (size == 6 && data[0] == 0x01) {

        atualizarMotores(
            data[1],
            data[2],
            data[3],
            data[4],
            data[5]
        );


        Serial.printf(
            "MOTORES -> M1:%3d M2:%3d M3:%3d M4:%3d M5:%3d\n",
            data[1],
            data[2],
            data[3],
            data[4],
            data[5]
        );

        return;
    }


    // --------------------------------------------------------
    // COMANDO DE EMERGENCY STOP
    //
    // [0x03]
    // --------------------------------------------------------

    if (size == 1 && data[0] == 0x03) {

        pararTodos();

        Serial.println(
            "!!! EMERGENCY STOP !!!"
        );

        return;
    }


    // --------------------------------------------------------
    // Comando inválido
    // --------------------------------------------------------

    Serial.printf(
        "Payload de motor invalido. Tamanho = %d\n",
        size
    );
}


// ============================================================
// 13. PROCESSAMENTO DO PACOTE CRTP
// ============================================================

void process_crtp_packet() {

    // --------------------------------------------------------
    // Extrai porta
    // --------------------------------------------------------

    uint8_t port =
        (rx_packet.header >> 4) & 0x0F;


    // --------------------------------------------------------
    // Extrai canal
    // --------------------------------------------------------

    uint8_t channel =
        rx_packet.header & 0x0F;


    Serial.printf(
        "\n=> PACOTE CRTP | Porta: %d | Canal: %d | Tamanho: %d\n",
        port,
        channel,
        rx_packet.size
    );


    printHexBuffer(
        rx_packet.data,
        rx_packet.size
    );


    // --------------------------------------------------------
    // Verifica se é o canal dos motores
    // --------------------------------------------------------

    if (
        port == CRTP_MOTOR_PORT &&
        channel == CRTP_MOTOR_CHANNEL
    ) {

        processarMotorPayload(
            rx_packet.data,
            rx_packet.size
        );


        // Atualiza watchdog SOMENTE depois de
        // receber um pacote válido de controle.

        ultimoPacoteValido = millis();


        // ACK

        send_crtp_ack(
            rx_packet.header
        );

    }

    else {

        Serial.printf(
            "Pacote ignorado: porta/canal nao correspondem "
            "ao controle dos motores.\n"
        );
    }
}


// ============================================================
// 14. MÁQUINA DE ESTADOS CRTP
// ============================================================

void process_crtp_byte(uint8_t byte) {

    switch (rx_state) {


        // ====================================================
        // ESPERA PELO BYTE 0xAA
        // ====================================================

        case STATE_WAIT_START:

            if (byte == CRTP_START_BYTE) {

                rx_state = STATE_HEADER;

            }

            break;


        // ====================================================
        // RECEBE HEADER
        // ====================================================

        case STATE_HEADER:

            // Se vier outro 0xAA, continua considerando
            // como possível início de novo pacote.

            if (byte == CRTP_START_BYTE) {

                break;

            }


            rx_packet.header = byte;

            calculated_checksum = byte;

            rx_state = STATE_SIZE;

            break;


        // ====================================================
        // RECEBE TAMANHO
        // ====================================================

        case STATE_SIZE:

            rx_packet.size = byte;

            calculated_checksum += byte;


            // Tamanho inválido

            if (
                rx_packet.size >
                CRTP_MAX_DATA_SIZE
            ) {

                Serial.println(
                    "Erro: tamanho de pacote CRTP invalido."
                );

                rx_state = STATE_WAIT_START;

            }


            // Pacote sem payload

            else if (
                rx_packet.size == 0
            ) {

                rx_state = STATE_CHECKSUM;

            }


            // Pacote com payload

            else {

                data_idx = 0;

                rx_state = STATE_DATA;

            }

            break;


        // ====================================================
        // RECEBE PAYLOAD
        // ====================================================

        case STATE_DATA:

            rx_packet.data[data_idx] = byte;

            data_idx++;

            calculated_checksum += byte;


            if (
                data_idx >=
                rx_packet.size
            ) {

                rx_state = STATE_CHECKSUM;

            }

            break;


        // ====================================================
        // RECEBE CHECKSUM
        // ====================================================

        case STATE_CHECKSUM:

            rx_packet.checksum = byte;


            // ------------------------------------------------
            // Verifica checksum
            // ------------------------------------------------

            if (
                calculated_checksum ==
                rx_packet.checksum
            ) {

                Serial.println(
                    "Checksum OK."
                );


                // Pacote válido

                process_crtp_packet();

            }

            else {

                Serial.printf(
                    "ERRO CHECKSUM | Calculado: 0x%02X | "
                    "Recebido: 0x%02X\n",
                    calculated_checksum,
                    rx_packet.checksum
                );

            }


            // Volta para procurar próximo pacote

            rx_state = STATE_WAIT_START;

            break;
    }
}


// ============================================================
// 15. WATCHDOG
// ============================================================

void verificarWatchdog() {

    // Se já recebemos pelo menos um pacote

    if (
        ultimoPacoteValido != 0
    ) {

        if (
            millis() -
            ultimoPacoteValido >
            CRTP_TIMEOUT_MS
        ) {

            // Para os motores

            pararTodos();


            Serial.println(
                "!!! TIMEOUT CRTP !!!"
            );

            Serial.println(
                "Motores desligados por perda de comunicacao."
            );


            // Zera para evitar ficar imprimindo
            // infinitamente

            ultimoPacoteValido = 0;
        }
    }
}


// ============================================================
// 16. SETUP
// ============================================================

void setup() {

    // --------------------------------------------------------
    // Serial USB para debug
    // --------------------------------------------------------

    Serial.begin(115200);

    delay(1000);


    // --------------------------------------------------------
    // Inicializa motores
    // --------------------------------------------------------

    configurarMotores();


    // --------------------------------------------------------
    // Inicializa UART CRTP
    // --------------------------------------------------------

    Serial1.begin(
        CRTP_BAUDRATE,
        SERIAL_8N1,
        CRTP_RX_PIN,
        CRTP_TX_PIN
    );


    // --------------------------------------------------------
    // Mensagens
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "=========================================="
    );

    Serial.println(
        "     RECEPTOR CRTP + 5 MOTORES"
    );

    Serial.println(
        "           XIAO ESP32-S3"
    );

    Serial.println(
        "=========================================="
    );

    Serial.println();


    Serial.println(
        "UART CRTP:"
    );

    Serial.println(
        "RX -> GPIO 44"
    );

    Serial.println(
        "TX -> GPIO 43"
    );

    Serial.println(
        "Baud -> 115200"
    );

    Serial.println();


    Serial.println(
        "CRTP:"
    );

    Serial.printf(
        "Porta -> %d\n",
        CRTP_MOTOR_PORT
    );

    Serial.printf(
        "Canal -> %d\n",
        CRTP_MOTOR_CHANNEL
    );

    Serial.println();


    Serial.println(
        "Motores:"
    );

    Serial.println(
        "M1 -> GPIO 1"
    );

    Serial.println(
        "M2 -> GPIO 3"
    );

    Serial.println(
        "M3 -> GPIO 4"
    );

    Serial.println(
        "M4 -> GPIO 9"
    );

    Serial.println(
        "M5 -> GPIO 7"
    );

    Serial.println();


    Serial.println(
        "Payload esperado:"
    );

    Serial.println(
        "[M1][M2][M3][M4][M5]"
    );

    Serial.println(
        "Cada motor: 0-255"
    );

    Serial.println();


    Serial.println(
        "Watchdog: 500 ms"
    );

    Serial.println();


    Serial.println(
        "Aguardando pacotes CRTP..."
    );

    Serial.println(
        "=========================================="
    );
}


// ============================================================
// 17. LOOP
// ============================================================

void loop() {

    // --------------------------------------------------------
    // Recebe todos os bytes disponíveis na UART CRTP
    // --------------------------------------------------------

    while (
        Serial1.available() > 0
    ) {

        uint8_t incoming_byte =
            Serial1.read();


        process_crtp_byte(
            incoming_byte
        );
    }


    // --------------------------------------------------------
    // Verifica perda de comunicação
    // --------------------------------------------------------

    verificarWatchdog();
}
```
