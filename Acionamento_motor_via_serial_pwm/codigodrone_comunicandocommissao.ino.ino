#include <Arduino.h>

// XIAO ESP32-S3 (RECEPTOR CRTP + 5 MOTORES)

// 1. PINOS DOS MOTORES
#define MOTOR1 1
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 9
#define MOTOR5 7

// 2. UART CRTP (PINOS PADRÃO DA XIAO ESP32-S3)
#define CRTP_RX_PIN 44
#define CRTP_TX_PIN 43
#define CRTP_BAUDRATE 115200

// 3. CRTP
#define CRTP_START_BYTE 0xAA
#define CRTP_COMMANDER_HEADER 0x30
#define CRTP_PAYLOAD_SIZE 14

// 4. PWM
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define PWM_MAX_LIMIT 80 // Limita corrente em 0,8

// 5. WATCHDOG (800ms cobre a latência de gravação de foto no SD pelo transmissor)
#define WATCHDOG_TIMEOUT 800

unsigned long ultimoPacoteValido = 0;

// 6. ESTRUTURA DO PAYLOAD
struct CommanderPayload
{
    float roll;
    float pitch;
    float yaw;
    uint16_t thrust;
} __attribute__((packed));

// 7. ESTADOS DO RECEPTOR
enum RxState
{
    WAIT_START_1,
    WAIT_START_2,
    WAIT_HEADER,
    WAIT_SIZE,
    WAIT_DATA,
    WAIT_CHECKSUM
};

RxState rxState = WAIT_START_1;

// 8. VARIÁVEIS DO PACOTE
uint8_t payload[CRTP_PAYLOAD_SIZE];
uint8_t payloadIndex = 0;
uint8_t calculatedChecksum = 0;

// 9. MOTORES
void pararTodos()
{
    ledcWrite(MOTOR1, 0);
    ledcWrite(MOTOR2, 0);
    ledcWrite(MOTOR3, 0);
    ledcWrite(MOTOR4, 0);
    ledcWrite(MOTOR5, 0);
}

void escreverMotores(uint8_t pwm)
{
    ledcWrite(MOTOR1, pwm);
    ledcWrite(MOTOR2, pwm);
    ledcWrite(MOTOR3, pwm);
    ledcWrite(MOTOR4, pwm);
    ledcWrite(MOTOR5, pwm);
}

// 10. CONVERSÃO THRUST -> PWM
uint8_t thrustParaPWM(uint16_t thrust)
{
    uint32_t pwm = ((uint32_t)thrust * (uint32_t)PWM_MAX_LIMIT) / 65535UL;

    if (pwm > PWM_MAX_LIMIT)
    {
        pwm = PWM_MAX_LIMIT;
    }

    return (uint8_t)pwm;
}

// 11. READY TO FLY
// Envia exatamente o pacote que o transmissor espera: AA AA F0 01 01 F2
void enviarReadyToFly()
{
    uint8_t pacote[6] = {0xAA, 0xAA, 0xF0, 0x01, 0x01, 0xF2};
    Serial1.write(pacote, 6);
    Serial1.flush();

    Serial.println();
    Serial.println(">>> READY TO FLY ENVIADO <<<");
    Serial.println("AA AA F0 01 01 F2");
}

// 12. PROCESSA PAYLOAD
void processarPayload()
{
    CommanderPayload comando;
    memcpy(&comando, payload, sizeof(CommanderPayload));

    ultimoPacoteValido = millis();

    uint8_t pwm = thrustParaPWM(comando.thrust);

    // THRUST = 0
    if (comando.thrust == 0)
    {
        pararTodos();
        Serial.println(">>> MOTORES DESLIGADOS <<<");
        return;
    }

    // ACIONA OS 5 MOTORES
    escreverMotores(pwm);
}

// 13. PROCESSA BYTE RECEBIDO
void processarByte(uint8_t byte)
{
    switch (rxState)
    {
        case WAIT_START_1:
            if (byte == CRTP_START_BYTE) rxState = WAIT_START_2;
            break;

        case WAIT_START_2:
            if (byte == CRTP_START_BYTE) rxState = WAIT_HEADER;
            else rxState = WAIT_START_1;
            break;

        case WAIT_HEADER:
            if (byte == CRTP_COMMANDER_HEADER)
            {
                calculatedChecksum = byte;
                rxState = WAIT_SIZE;
            }
            else
            {
                rxState = WAIT_START_1;
            }
            break;

        case WAIT_SIZE:
            if (byte == CRTP_PAYLOAD_SIZE)
            {
                calculatedChecksum += byte;
                payloadIndex = 0;
                rxState = WAIT_DATA;
            }
            else
            {
                rxState = WAIT_START_1;
            }
            break;

        case WAIT_DATA:
            payload[payloadIndex] = byte;
            calculatedChecksum += byte;
            payloadIndex++;

            if (payloadIndex >= CRTP_PAYLOAD_SIZE)
            {
                rxState = WAIT_CHECKSUM;
            }
            break;

        case WAIT_CHECKSUM:
            if (byte == calculatedChecksum)
            {
                processarPayload();
            }
            else
            {
                Serial.println("!!! CHECKSUM INVALIDO !!!");
            }
            rxState = WAIT_START_1;
            break;
    }
}

// 14. WATCHDOG
void verificarWatchdog()
{
    if (ultimoPacoteValido != 0 && (millis() - ultimoPacoteValido > WATCHDOG_TIMEOUT))
    {
        pararTodos();
        ultimoPacoteValido = 0;

        Serial.println();
        Serial.println("!!! WATCHDOG: Perda de comunicacao. Motores desligados. !!!");
    }
}

void setup()
{
    Serial.begin(115200);

    // Configura e anexa PWM aos pinos dos motores
    ledcAttach(MOTOR1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR3, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR4, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR5, PWM_FREQ, PWM_RESOLUTION);

    pararTodos();

    // Inicializa a UART CRTP
    Serial1.begin(CRTP_BAUDRATE, SERIAL_8N1, CRTP_RX_PIN, CRTP_TX_PIN);

    Serial.println("\n--- RECEPTOR INICIADO ---");
    Serial.println("Aguardando 4.5s para boot do Transmissor (Câmera + SD + PSRAM)...");

    delay(1000);

    // Envia o Ready To Fly uma única vez
    enviarReadyToFly();

    Serial.println("Aguardando comandos...");
}

void loop()
{
    while (Serial1.available() > 0)
    {
        uint8_t byte = Serial1.read();
        processarByte(byte);
    }

    verificarWatchdog();
}
