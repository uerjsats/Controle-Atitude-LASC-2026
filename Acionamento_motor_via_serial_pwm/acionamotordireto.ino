#include <Arduino.h>

// ============================================================
// XIAO ESP32-S3
// RECEPTOR CRTP + 5 MOTORES
//
// Compatível com o transmissor fornecido
//
// UART:
// RX = GPIO 44
// TX = GPIO 43
// Baud = 115200
//
// COMANDO DO TRANSMISSOR:
// Header = 0x30
// Porta  = 3
// Canal = 0
//
// PAYLOAD:
// float roll       -> 4 bytes
// float pitch      -> 4 bytes
// float yaw        -> 4 bytes
// uint16_t thrust  -> 2 bytes
//
// TOTAL PAYLOAD = 14 bytes
//
// PWM:
// Frequência = 20 kHz
// Resolução  = 8 bits
//
// MOTORES:
// M1 -> GPIO 1
// M2 -> GPIO 3
// M3 -> GPIO 4
// M4 -> GPIO 9
// M5 -> GPIO 7
// ============================================================


// ============================================================
// 1. PINOS DOS MOTORES
// ============================================================

#define MOTOR1 1
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 9
#define MOTOR5 7


// ============================================================
// 2. UART
// ============================================================

#define TX_PIN 43
#define RX_PIN 44
#define BAUDRATE 115200


// ============================================================
// 3. CRTP
// ============================================================

#define CRTP_START_BYTE 0xAA

// Comando enviado pelo seu transmissor
#define COMMANDER_HEADER 0x30

// Tamanho do payload
#define COMMANDER_PAYLOAD_SIZE 14


// ============================================================
// 4. PWM
// ============================================================

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8


// ============================================================
// 5. WATCHDOG
// ============================================================

// Se ficar mais de 500 ms sem receber comando,
// todos os motores são desligados.

#define COMMAND_TIMEOUT 500

unsigned long lastCommandTime = 0;


// ============================================================
// 6. PAYLOAD DO TRANSMISSOR
// ============================================================

struct CommanderPayload {

    float roll;

    float pitch;

    float yaw;

    uint16_t thrust;

} _attribute_((packed));


// ============================================================
// 7. VARIÁVEIS RECEBIDAS
// ============================================================

float receivedRoll = 0;

float receivedPitch = 0;

float receivedYaw = 0;

uint16_t receivedThrust = 0;


// ============================================================
// 8. MÁQUINA DE ESTADOS
// ============================================================

enum RXState {

    WAIT_START_1,
    WAIT_START_2,
    WAIT_HEADER,
    WAIT_SIZE,
    READ_PAYLOAD,
    WAIT_CHECKSUM

};

RXState rxState = WAIT_START_1;


// ============================================================
// 9. BUFFER
// ============================================================

uint8_t payloadBuffer[COMMANDER_PAYLOAD_SIZE];

uint8_t payloadIndex = 0;

uint8_t calculatedChecksum = 0;


// ============================================================
// 10. READY TO FLY
// ============================================================

bool readyToFly = false;


// ============================================================
// 11. CONFIGURAÇÃO DOS MOTORES
// ============================================================

void configurarMotores() {

    // PWM = 20 kHz
    // resolução = 8 bits

    ledcAttach(
        MOTOR1,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttach(
        MOTOR2,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttach(
        MOTOR3,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttach(
        MOTOR4,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttach(
        MOTOR5,
        PWM_FREQ,
        PWM_RESOLUTION
    );


    // Segurança
    pararTodos();
}


// ============================================================
// 12. DESLIGA TODOS OS MOTORES
// ============================================================

void pararTodos() {

    ledcWrite(MOTOR1, 0);

    ledcWrite(MOTOR2, 0);

    ledcWrite(MOTOR3, 0);

    ledcWrite(MOTOR4, 0);

    ledcWrite(MOTOR5, 0);
}


// ============================================================
// 13. ACIONA OS MOTORES
// ============================================================

void atualizarMotores(uint8_t pwm) {

    // Neste teste, todos os motores recebem
    // o mesmo PWM.

    ledcWrite(
        MOTOR1,
        pwm
    );

    ledcWrite(
        MOTOR2,
        pwm
    );

    ledcWrite(
        MOTOR3,
        pwm
    );

    ledcWrite(
        MOTOR4,
        pwm
    );

    ledcWrite(
        MOTOR5,
        pwm
    );
}


// ============================================================
// 14. CONVERTE THRUST PARA PWM
// ============================================================
//
// Seu transmissor usa:
//
// 0       -> mínimo
// 32767   -> hover
// 45000   -> takeoff
//
// O PWM é:
//
// 0       -> motor desligado
// 255     -> máximo
//
// Como uint16_t vai até 65535,
// fazemos a conversão proporcional.
//
// ============================================================

uint8_t thrustParaPWM(uint16_t thrust) {

    // Segurança

    if (thrust == 0) {

        return 0;
    }


    // Limita o valor máximo

    if (thrust > 65535) {

        thrust = 65535;
    }


    uint32_t pwm =
        ((uint32_t)thrust * 255) / 65535;


    return (uint8_t)pwm;
}


// ============================================================
// 15. PROCESSA PAYLOAD
// ============================================================

void processarPayload() {

    CommanderPayload payload;


    // Copia os 14 bytes para a estrutura

    memcpy(
        &payload,
        payloadBuffer,
        sizeof(payload)
    );


    // Guarda os valores

    receivedRoll =
        payload.roll;

    receivedPitch =
        payload.pitch;

    receivedYaw =
        payload.yaw;

    receivedThrust =
        payload.thrust;


    // ========================================================
    // CONVERTE THRUST PARA PWM
    // ========================================================

    uint8_t pwm =
        thrustParaPWM(
            receivedThrust
        );


    // ========================================================
    // ACIONA MOTORES
    // ========================================================

    atualizarMotores(pwm);


    // ========================================================
    // ATUALIZA WATCHDOG
    // ========================================================

    lastCommandTime =
        millis();


    // ========================================================
    // MOSTRA DADOS
    // ========================================================

    Serial.println(
        "------------------------------"
    );

    Serial.printf(
        "ROLL   : %.2f\n",
        receivedRoll
    );

    Serial.printf(
        "PITCH  : %.2f\n",
        receivedPitch
    );

    Serial.printf(
        "YAW    : %.2f\n",
        receivedYaw
    );

    Serial.printf(
        "THRUST : %u\n",
        receivedThrust
    );

    Serial.printf(
        "PWM    : %u / 255\n",
        pwm
    );

    Serial.println(
        "------------------------------"
    );
}


// ============================================================
// 16. PROCESSA BYTE RECEBIDO
// ============================================================

void processarByte(uint8_t c) {

    switch (rxState) {


        // ====================================================
        // ESPERA PRIMEIRO AA
        // ====================================================

        case WAIT_START_1:

            if (
                c == CRTP_START_BYTE
            ) {

                rxState =
                    WAIT_START_2;
            }

            break;


        // ====================================================
        // ESPERA SEGUNDO AA
        // ====================================================

        case WAIT_START_2:

            if (
                c == CRTP_START_BYTE
            ) {

                rxState =
                    WAIT_HEADER;
            }

            else {

                rxState =
                    WAIT_START_1;
            }

            break;


        // ====================================================
        // HEADER
        // ====================================================

        case WAIT_HEADER:

            if (
                c == COMMANDER_HEADER
            ) {

                calculatedChecksum =
                    c;

                rxState =
                    WAIT_SIZE;
            }

            else {

                rxState =
                    WAIT_START_1;
            }

            break;


        // ====================================================
        // SIZE
        // ====================================================

        case WAIT_SIZE:

            if (
                c ==
                COMMANDER_PAYLOAD_SIZE
            ) {

                calculatedChecksum +=
                    c;

                payloadIndex = 0;

                rxState =
                    READ_PAYLOAD;
            }

            else {

                Serial.printf(
                    "Tamanho inesperado: %d\n",
                    c
                );

                rxState =
                    WAIT_START_1;
            }

            break;


        // ====================================================
        // PAYLOAD
        // ====================================================

        case READ_PAYLOAD:

            payloadBuffer[
                payloadIndex
            ] = c;


            calculatedChecksum += c;

            payloadIndex++;


            if (
                payloadIndex >=
                COMMANDER_PAYLOAD_SIZE
            ) {

                rxState =
                    WAIT_CHECKSUM;
            }

            break;


        // ====================================================
        // CHECKSUM
        // ====================================================

        case WAIT_CHECKSUM:

            if (
                c ==
                calculatedChecksum
            ) {

                Serial.println(
                    "CRTP OK"
                );


                processarPayload();
            }

            else {

                Serial.printf(

                    "ERRO CHECKSUM | "
                    "Calculado: 0x%02X | "
                    "Recebido: 0x%02X\n",

                    calculatedChecksum,
                    c

                );
            }


            rxState =
                WAIT_START_1;

            break;
    }
}


// ============================================================
// 17. WATCHDOG
// ============================================================

void verificarWatchdog() {

    // Se nunca recebeu comando,
    // não faz nada.

    if (
        lastCommandTime == 0
    ) {

        return;
    }


    // Verifica timeout

    if (
        millis() -
        lastCommandTime >
        COMMAND_TIMEOUT
    ) {

        pararTodos();


        Serial.println(
            "!!! TIMEOUT !!!"
        );

        Serial.println(
            "Motores desligados."
        );


        // Evita repetir a mensagem

        lastCommandTime = 0;
    }
}


// ============================================================
// 18. SETUP
// ============================================================

void setup() {

    // --------------------------------------------------------
    // SERIAL USB
    // --------------------------------------------------------

    Serial.begin(115200);

    delay(1000);


    // --------------------------------------------------------
    // MOTORES
    // --------------------------------------------------------

    configurarMotores();


    // --------------------------------------------------------
    // UART CRTP
    // --------------------------------------------------------

    Serial1.begin(

        BAUDRATE,
        SERIAL_8N1,
        RX_PIN,
        TX_PIN

    );


    // --------------------------------------------------------
    // MENSAGENS
    // --------------------------------------------------------

    Serial.println();

    Serial.println(
        "======================================"
    );

    Serial.println(
        " XIAO ESP32-S3 - RECEPTOR CRTP"
    );

    Serial.println(
        " CONTROLE DE 5 MOTORES"
    );

    Serial.println(
        "======================================"
    );

    Serial.println();

    Serial.println(
        "UART:"
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
        "CRTP HEADER -> 0x30"
    );

    Serial.println(
        "PAYLOAD -> 14 bytes"
    );

    Serial.println();

    Serial.println(
        "PWM:"
    );

    Serial.println(
        "Frequencia -> 20 kHz"
    );

    Serial.println(
        "Resolucao -> 8 bits"
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
        "Aguardando comandos CRTP..."
    );

    Serial.println(
        "======================================"
    );
}


// ============================================================
// 19. LOOP
// ============================================================

void loop() {

    // --------------------------------------------------------
    // Recebe dados da UART
    // --------------------------------------------------------

    while (
        Serial1.available() > 0
    ) {

        uint8_t c =
            Serial1.read();


        processarByte(c);
    }


    // --------------------------------------------------------
    // Watchdog
    // --------------------------------------------------------

    verificarWatchdog();
}
