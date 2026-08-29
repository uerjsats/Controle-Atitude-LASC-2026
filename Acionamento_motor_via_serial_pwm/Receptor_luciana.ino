#include <Arduino.h>

// XIAO ESP32-S3 (RECEPTOR CRTP + 5 MOTORES)

// 1. PINOS DOS MOTORES
#define MOTOR1 1
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 9
#define MOTOR5 7

// 2. UART CRTP
#define CRTP_RX_PIN 44
#define CRTP_TX_PIN 43
#define CRTP_BAUDRATE 115200

// 3. CRTP
#define CRTP_START_BYTE 0xAA

// Header enviado pelo transmissor
#define CRTP_COMMANDER_HEADER 0x30

// Payload:
#define CRTP_PAYLOAD_SIZE 14

// 4. PWM
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
// PWM possui 8 bits: 0 -> 255
#define PWM_MAX_LIMIT 178 

// 5. WATCHDOG
#define WATCHDOG_TIMEOUT 300

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
// O transmissor usa: 0 -> motor desligado, 32767   -> aproximadamente 50%, 65535   -> 100%
uint8_t thrustParaPWM(uint16_t thrust)
{
    uint32_t pwm;

    // Mapeia 0..65535 diretamente para 0..PWM_MAX_LIMIT
    pwm = ((uint32_t)thrust * (uint32_t)PWM_MAX_LIMIT) / 65535UL;

    if (pwm > PWM_MAX_LIMIT)
    {
        pwm = PWM_MAX_LIMIT;
    }

    return (uint8_t)pwm;
}

// 11. READY TO FLY
// O transmissor espera: AA AA F0 01 01 F2
void enviarReadyToFly()
{
    uint8_t pacote[6];

    pacote[0] = 0xAA;
    pacote[1] = 0xAA;
    pacote[2] = 0xF0;
    pacote[3] = 0x01;
    pacote[4] = 0x01;
    pacote[5] = 0xF2;

    Serial1.write( pacote, 6);
    Serial1.flush();

    Serial.println();
    Serial.println( ">>> READY TO FLY ENVIADO <<<" );

    Serial.println( "AA AA F0 01 01 F2");
}

// 12. PROCESSA PAYLOAD
void processarPayload()
{
    CommanderPayload comando;

    // Copia os 14 bytes recebidos
    memcpy( &comando, payload, sizeof(CommanderPayload));

    // Atualiza watchdog
    ultimoPacoteValido = millis();

    // Converte thrust
    uint8_t pwm = thrustParaPWM(comando.thrust);

    // MOSTRA DADOS
    Serial.print("ROLL: ");
    Serial.print(comando.roll, 2);

    Serial.print(" | PITCH: ");
    Serial.print(comando.pitch, 2);

    Serial.print(" | YAW: ");
    Serial.print(comando.yaw, 2);

    Serial.print(" | THRUST: ");
    Serial.print(comando.thrust);

    Serial.print(" | PWM: ");
    Serial.println(pwm);

    // THRUST = 0
    if (comando.thrust == 0)
    {
        pararTodos();

        Serial.println( ">>> MOTORES DESLIGADOS <<<" );

        return;
    }

    // ACIONA OS CINCO MOTORES
    escreverMotores(pwm);
}

// 13. PROCESSA BYTE RECEBIDO
void processarByte(uint8_t byte)
{
    switch (rxState)
    {
        // ESPERA PRIMEIRO AA
        case WAIT_START_1:

            if (byte == CRTP_START_BYTE)
            {
                rxState = WAIT_START_2;
            }

            break;

        // ESPERA SEGUNDO AA
        case WAIT_START_2:

            if (byte == CRTP_START_BYTE)
            {
                rxState = WAIT_HEADER;
            }
            else
            {
                rxState = WAIT_START_1;
            }

            break;

        // HEADER
        case WAIT_HEADER:

            if ( byte == CRTP_COMMANDER_HEADER )
            {
                calculatedChecksum = byte;

                rxState = WAIT_SIZE;
            }
            else
            {
                rxState = WAIT_START_1;
            }

            break;

        // SIZE
        case WAIT_SIZE:

            if ( byte == CRTP_PAYLOAD_SIZE)
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

        // PAYLOAD
        case WAIT_DATA:

            payload[payloadIndex] = byte;

            calculatedChecksum += byte;

            payloadIndex++;

            if ( payloadIndex >= CRTP_PAYLOAD_SIZE)
            {
                rxState = WAIT_CHECKSUM;
            }

            break;

        // CHECKSUM
        case WAIT_CHECKSUM:

            if ( byte == calculatedChecksum)
            {
                processarPayload();
            }
            else
            {
                Serial.println( "!!! CHECKSUM INVALIDO !!!" );
            }

            rxState = WAIT_START_1;
            break;
    }
}

// 14. WATCHDOG
void verificarWatchdog()
{
    if ( ultimoPacoteValido != 0 && millis() - ultimoPacoteValido > WATCHDOG_TIMEOUT)
    {
        pararTodos();

        ultimoPacoteValido = 0;

        Serial.println();
        Serial.println( "!!! WATCHDOG !!!" );

        Serial.println( "Comunicacao perdida.");

        Serial.println( "Motores desligados.");
    }
}

// 15. SETUP
void setup()
{
    Serial.begin(115200);

    delay(1000);

    // PWM
    bool ok1 = ledcAttach( MOTOR1, PWM_FREQ, PWM_RESOLUTION);

    bool ok2 = ledcAttach( MOTOR2, PWM_FREQ, PWM_RESOLUTION);

    bool ok3 = ledcAttach( MOTOR3, PWM_FREQ, PWM_RESOLUTION);

    bool ok4 = ledcAttach( MOTOR4, PWM_FREQ, PWM_RESOLUTION);

    bool ok5 = ledcAttach( MOTOR5, PWM_FREQ, PWM_RESOLUTION);

    // VERIFICA PWM
    Serial.println();

    Serial.print("PWM M1: ");
    Serial.println(ok1 ? "OK" : "ERRO");

    Serial.print("PWM M2: ");
    Serial.println(ok2 ? "OK" : "ERRO");

    Serial.print("PWM M3: ");
    Serial.println(ok3 ? "OK" : "ERRO");

    Serial.print("PWM M4: ");
    Serial.println(ok4 ? "OK" : "ERRO");

    Serial.print("PWM M5: ");
    Serial.println(ok5 ? "OK" : "ERRO");

    // DESLIGA TUDO
    pararTodos();

    // UART CRTP
    Serial1.begin( CRTP_BAUDRATE, SERIAL_8N1, CRTP_RX_PIN, CRTP_TX_PIN);

    // INFORMAÇÕES
    Serial.println();

    Serial.println( "UART CRTP:");

    Serial.println("RX = GPIO 44");

    Serial.println( "TX = GPIO 43");

    Serial.println("Baud = 115200");

    Serial.println();

    Serial.println("MOTORES:");

    Serial.println("M1 = GPIO 1");

    Serial.println("M2 = GPIO 3");

    Serial.println("M3 = GPIO 4");

    Serial.println("M4 = GPIO 9");

    Serial.println("M5 = GPIO 7");

    Serial.println();

    Serial.println("PWM = 20000 Hz");

    Serial.println("Resolucao = 8 bits");

    // ENVIA READY TO FLY
    delay(1000);

    enviarReadyToFly();

    Serial.println();

    Serial.println("Aguardando comandos...");
    
}

// 16. LOOP
void loop()
{
    while ( Serial1.available() > 0 )
    {
        uint8_t byte = Serial1.read();

        processarByte(byte);
    }

    // WATCHDOG
    verificarWatchdog();
}
