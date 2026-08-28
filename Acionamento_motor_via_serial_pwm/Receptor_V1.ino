#include <Arduino.h>

// ============================================================
// XIAO ESP32-S3 - RECEPTOR
// CRTP COMMANDER + 5 MOTORES
// ============================================================

// -------------------- MOTORES -------------------------------

#define MOTOR1 1
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 9
#define MOTOR5 7


// -------------------- UART -----------------------------------

#define RX_PIN 44
#define TX_PIN 43

#define BAUDRATE 115200


// -------------------- CRTP -----------------------------------

#define START_BYTE 0xAA
#define COMMANDER_HEADER 0x30

#define PAYLOAD_SIZE 14


// -------------------- PWM ------------------------------------

#define PWM_FREQ 20000
#define PWM_RES 8


// -------------------- WATCHDOG ------------------------------

#define WATCHDOG_TIMEOUT 300

unsigned long ultimoPacote = 0;


// ============================================================
// ESTRUTURA DO COMMANDER
// ============================================================

struct CommanderPayload
{
    float roll;
    float pitch;
    float yaw;
    uint16_t thrust;
} __attribute__((packed));


// ============================================================
// RECEPÇÃO
// ============================================================

enum EstadoRX
{
    WAIT_AA1,
    WAIT_AA2,
    WAIT_HEADER,
    WAIT_SIZE,
    RECEIVE_DATA,
    WAIT_CHECKSUM
};

EstadoRX estadoRX = WAIT_AA1;

uint8_t buffer[PAYLOAD_SIZE];

uint8_t indice = 0;

uint8_t checksum = 0;


// ============================================================
// PWM
// ============================================================

void motoresOff()
{
    ledcWrite(MOTOR1, 0);
    ledcWrite(MOTOR2, 0);
    ledcWrite(MOTOR3, 0);
    ledcWrite(MOTOR4, 0);
    ledcWrite(MOTOR5, 0);
}


void motoresPWM(uint8_t pwm)
{
    ledcWrite(MOTOR1, pwm);
    ledcWrite(MOTOR2, pwm);
    ledcWrite(MOTOR3, pwm);
    ledcWrite(MOTOR4, pwm);
    ledcWrite(MOTOR5, pwm);
}


// ============================================================
// CONVERSÃO THRUST -> PWM
// ============================================================

uint8_t converterThrust(uint16_t thrust)
{
    uint32_t pwm;

    pwm = ((uint32_t)thrust * 255UL) / 65535UL;

    if (pwm > 255)
        pwm = 255;

    return (uint8_t)pwm;
}


// ============================================================
// PROCESSA COMANDO
// ============================================================

void processarComando()
{
    CommanderPayload cmd;

    memcpy(
        &cmd,
        buffer,
        sizeof(cmd)
    );


    // --------------------------------------------------------
    // Atualiza watchdog
    // --------------------------------------------------------

    ultimoPacote = millis();


    // --------------------------------------------------------
    // Converte thrust
    // --------------------------------------------------------

    uint8_t pwm =
        converterThrust(cmd.thrust);


    // --------------------------------------------------------
    // Mostra tudo no Serial
    // --------------------------------------------------------

    Serial.println();
    Serial.println("--------------------------------");

    Serial.print("ROLL   = ");
    Serial.println(cmd.roll);

    Serial.print("PITCH  = ");
    Serial.println(cmd.pitch);

    Serial.print("YAW    = ");
    Serial.println(cmd.yaw);

    Serial.print("THRUST = ");
    Serial.println(cmd.thrust);

    Serial.print("PWM    = ");
    Serial.println(pwm);


    // --------------------------------------------------------
    // Acionamento
    // --------------------------------------------------------

    if (cmd.thrust == 0)
    {
        motoresOff();

        Serial.println("MOTORES = OFF");
    }
    else
    {
        motoresPWM(pwm);

        Serial.println("MOTORES = ON");
    }

    Serial.println("--------------------------------");
}


// ============================================================
// PROCESSA BYTE
// ============================================================

void processarByte(uint8_t c)
{
    switch (estadoRX)
    {
        // ----------------------------------------------------
        // PRIMEIRO AA
        // ----------------------------------------------------

        case WAIT_AA1:

            if (c == START_BYTE)
            {
                estadoRX = WAIT_AA2;
            }

            break;


        // ----------------------------------------------------
        // SEGUNDO AA
        // ----------------------------------------------------

        case WAIT_AA2:

            if (c == START_BYTE)
            {
                estadoRX = WAIT_HEADER;
            }
            else
            {
                estadoRX = WAIT_AA1;
            }

            break;


        // ----------------------------------------------------
        // HEADER
        // ----------------------------------------------------

        case WAIT_HEADER:

            if (c == COMMANDER_HEADER)
            {
                checksum = c;

                estadoRX = WAIT_SIZE;
            }
            else
            {
                estadoRX = WAIT_AA1;
            }

            break;


        // ----------------------------------------------------
        // SIZE
        // ----------------------------------------------------

        case WAIT_SIZE:

            if (c == PAYLOAD_SIZE)
            {
                checksum += c;

                indice = 0;

                estadoRX = RECEIVE_DATA;
            }
            else
            {
                estadoRX = WAIT_AA1;
            }

            break;


        // ----------------------------------------------------
        // PAYLOAD
        // ----------------------------------------------------

        case RECEIVE_DATA:

            buffer[indice] = c;

            checksum += c;

            indice++;


            if (indice >= PAYLOAD_SIZE)
            {
                estadoRX = WAIT_CHECKSUM;
            }

            break;


        // ----------------------------------------------------
        // CHECKSUM
        // ----------------------------------------------------

        case WAIT_CHECKSUM:

            if (c == checksum)
            {
                processarComando();
            }
            else
            {
                Serial.println(
                    "ERRO: CHECKSUM"
                );
            }

            estadoRX = WAIT_AA1;

            break;
    }
}


// ============================================================
// READY TO FLY
// ============================================================

void enviarReadyToFly()
{
    uint8_t pacote[6];

    pacote[0] = 0xAA;
    pacote[1] = 0xAA;
    pacote[2] = 0xF0;
    pacote[3] = 0x01;
    pacote[4] = 0x01;
    pacote[5] = 0xF2;

    Serial1.write(
        pacote,
        6
    );

    Serial1.flush();

    Serial.println();
    Serial.println(
        "READY TO FLY ENVIADO"
    );

    Serial.println(
        "AA AA F0 01 01 F2"
    );
}


// ============================================================
// WATCHDOG
// ============================================================

void verificarWatchdog()
{
    if (
        ultimoPacote != 0 &&
        millis() - ultimoPacote >
        WATCHDOG_TIMEOUT
    )
    {
        motoresOff();

        ultimoPacote = 0;

        Serial.println();
        Serial.println(
            "!!! WATCHDOG !!!"
        );

        Serial.println(
            "COMUNICACAO PERDIDA"
        );

        Serial.println(
            "MOTORES DESLIGADOS"
        );
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);


    // ========================================================
    // PWM
    // ========================================================

    ledcAttach(
        MOTOR1,
        PWM_FREQ,
        PWM_RES
    );

    ledcAttach(
        MOTOR2,
        PWM_FREQ,
        PWM_RES
    );

    ledcAttach(
        MOTOR3,
        PWM_FREQ,
        PWM_RES
    );

    ledcAttach(
        MOTOR4,
        PWM_FREQ,
        PWM_RES
    );

    ledcAttach(
        MOTOR5,
        PWM_FREQ,
        PWM_RES
    );


    // ========================================================
    // DESLIGA MOTORES
    // ========================================================

    motoresOff();


    // ========================================================
    // UART
    // ========================================================

    Serial1.begin(
        BAUDRATE,
        SERIAL_8N1,
        RX_PIN,
        TX_PIN
    );


    // ========================================================
    // INFORMAÇÕES
    // ========================================================

    Serial.println();
    Serial.println(
        "======================================"
    );

    Serial.println(
        " XIAO ESP32-S3 - RECEPTOR CRTP"
    );

    Serial.println(
        "======================================"
    );

    Serial.println();

    Serial.println(
        "PWM = 20000 Hz"
    );

    Serial.println(
        "Resolucao = 8 bits"
    );

    Serial.println();

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


    // ========================================================
    // READY TO FLY
    // ========================================================

    delay(1000);

    enviarReadyToFly();

    Serial.println();

    Serial.println(
        "AGUARDANDO COMMANDER..."
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Recebe dados
    // --------------------------------------------------------

    while (
        Serial1.available() > 0
    )
    {
        uint8_t c =
            Serial1.read();

        processarByte(c);
    }


    // --------------------------------------------------------
    // Watchdog
    // --------------------------------------------------------

    verificarWatchdog();
}