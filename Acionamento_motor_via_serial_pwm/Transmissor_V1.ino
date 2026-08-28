#include <Arduino.h>

// ============================================================
// XIAO ESP32-S3
// TRANSMISSOR CRTP - MISSÃO AUTÔNOMA
//
// UART:
// RX = GPIO 44
// TX = GPIO 43
// Baud = 115200
//
// CRTP COMMANDER:
// Header = 0x30
//
// Payload:
// roll   = float
// pitch  = float
// yaw    = float
// thrust = uint16_t
// ============================================================


// ============================================================
// UART
// ============================================================

#define TX_PIN 43
#define RX_PIN 44
#define BAUDRATE 115200


// ============================================================
// CRTP
// ============================================================

#define CRTP_START_BYTE 0xAA

#define COMMANDER_HEADER 0x30


// ============================================================
// PAYLOAD
// ============================================================

struct CommanderPayload
{
    float roll;
    float pitch;
    float yaw;
    uint16_t thrust;
} __attribute__((packed));


// ============================================================
// TEMPORIZAÇÃO
// ============================================================

// 20 ms = 50 Hz

unsigned long lastCommandTime = 0;

const unsigned long COMMAND_INTERVAL = 20;


// ============================================================
// TEMPORIZAÇÃO DOS ESTADOS
// ============================================================

// Cada estado dura 10 segundos

unsigned long stateStartTime = 0;

const unsigned long STATE_DURATION = 10000;


// ============================================================
// READY TO FLY
// ============================================================

bool is_ready_to_fly = false;


// ============================================================
// ESTADOS DA MISSÃO
// ============================================================

enum FlightState
{
    TAKEOFF,

    HOVER_1,

    MOVING_1,

    HOVER_2,

    MOVING_2,

    HOVER_3,

    LANDING,

    MISSION_FINISHED
};


FlightState currentState = TAKEOFF;


// ============================================================
// COMANDOS ATUAIS
// ============================================================

float current_roll = 0.0;

float current_pitch = 0.0;

float current_yaw = 0.0;

uint16_t current_thrust = 0;


// ============================================================
// TRANSMISSÃO CRTP
// ============================================================

void send_crtp_command(
    float roll,
    float pitch,
    float yaw,
    uint16_t thrust
)
{
    CommanderPayload payload;

    payload.roll = roll;

    payload.pitch = pitch;

    payload.yaw = yaw;

    payload.thrust = thrust;


    uint8_t payload_size =
        sizeof(payload);


    uint8_t checksum = 0;


    // --------------------------------------------------------
    // START
    // --------------------------------------------------------

    Serial1.write(
        CRTP_START_BYTE
    );

    Serial1.write(
        CRTP_START_BYTE
    );


    // --------------------------------------------------------
    // HEADER
    // --------------------------------------------------------

    Serial1.write(
        COMMANDER_HEADER
    );

    checksum +=
        COMMANDER_HEADER;


    // --------------------------------------------------------
    // SIZE
    // --------------------------------------------------------

    Serial1.write(
        payload_size
    );

    checksum +=
        payload_size;


    // --------------------------------------------------------
    // PAYLOAD
    // --------------------------------------------------------

    uint8_t* payload_bytes =
        (uint8_t*)&payload;


    for (
        uint8_t i = 0;
        i < payload_size;
        i++
    )
    {
        Serial1.write(
            payload_bytes[i]
        );

        checksum +=
            payload_bytes[i];
    }


    // --------------------------------------------------------
    // CHECKSUM
    // --------------------------------------------------------

    Serial1.write(
        checksum
    );
}


// ============================================================
// RECEPÇÃO DO READY TO FLY
// ============================================================

void process_rx_ready_to_fly()
{
    static uint8_t rx_state = 0;

    static uint8_t calculated_checksum = 0;


    while (
        Serial1.available() > 0 &&
        !is_ready_to_fly
    )
    {
        uint8_t c =
            Serial1.read();


        switch (rx_state)
        {

            // =================================================
            // PRIMEIRO AA
            // =================================================

            case 0:

                if (
                    c == CRTP_START_BYTE
                )
                {
                    rx_state = 1;
                }

                break;


            // =================================================
            // SEGUNDO AA
            // =================================================

            case 1:

                if (
                    c == CRTP_START_BYTE
                )
                {
                    rx_state = 2;
                }
                else
                {
                    rx_state = 0;
                }

                break;


            // =================================================
            // HEADER F0
            // =================================================

            case 2:

                if (c == 0xF0)
                {
                    calculated_checksum =
                        c;

                    rx_state = 3;
                }
                else
                {
                    rx_state = 0;
                }

                break;


            // =================================================
            // SIZE = 1
            // =================================================

            case 3:

                if (c == 0x01)
                {
                    calculated_checksum += c;

                    rx_state = 4;
                }
                else
                {
                    rx_state = 0;
                }

                break;


            // =================================================
            // PAYLOAD = 1
            // =================================================

            case 4:

                if (c == 0x01)
                {
                    calculated_checksum += c;

                    rx_state = 5;
                }
                else
                {
                    rx_state = 0;
                }

                break;


            // =================================================
            // CHECKSUM
            // =================================================

            case 5:

                if (
                    c == calculated_checksum
                )
                {
                    is_ready_to_fly = true;


                    Serial.println();
                    Serial.println(
                        "================================"
                    );

                    Serial.println(
                        "READY TO FLY RECEBIDO!"
                    );

                    Serial.println(
                        "INICIANDO MISSAO"
                    );

                    Serial.println(
                        "-> TAKEOFF"
                    );

                    Serial.println(
                        "================================"
                    );


                    // ------------------------------------------------
                    // Sincroniza relógios
                    // ------------------------------------------------

                    stateStartTime =
                        millis();

                    lastCommandTime =
                        millis();
                }
                else
                {
                    Serial.println(
                        "Checksum RTF invalido."
                    );
                }


                rx_state = 0;

                break;
        }
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // USB SERIAL
    // --------------------------------------------------------

    Serial.begin(
        115200
    );


    delay(1000);


    Serial.println();
    Serial.println(
        "======================================"
    );

    Serial.println(
        " XIAO ESP32-S3 - TRANSMISSOR CRTP"
    );

    Serial.println(
        "======================================"
    );

    Serial.println();


    // --------------------------------------------------------
    // UART CRTP
    // --------------------------------------------------------

    Serial1.begin(
        BAUDRATE,
        SERIAL_8N1,
        RX_PIN,
        TX_PIN
    );


    Serial.println(
        "UART CRTP:"
    );

    Serial.println(
        "TX -> GPIO 43"
    );

    Serial.println(
        "RX -> GPIO 44"
    );

    Serial.println(
        "Baud -> 115200"
    );

    Serial.println();


    Serial.println(
        "Aguardando READY TO FLY..."
    );

    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // 1. ESPERA READY TO FLY
    // ========================================================

    if (!is_ready_to_fly)
    {
        process_rx_ready_to_fly();

        return;
    }


    // ========================================================
    // 2. TEMPO ATUAL
    // ========================================================

    unsigned long currentMillis =
        millis();


    unsigned long timeInState =
        currentMillis -
        stateStartTime;


    // ========================================================
    // 3. MUDANÇA DE ESTADO
    // ========================================================

    if (
        currentState !=
        MISSION_FINISHED
    )
    {
        if (
            timeInState >=
            STATE_DURATION
        )
        {
            stateStartTime =
                currentMillis;

            timeInState = 0;


            switch (currentState)
            {

                case TAKEOFF:

                    currentState =
                        HOVER_1;

                    Serial.println(
                        "-> HOVER_1"
                    );

                    break;


                case HOVER_1:

                    currentState =
                        MOVING_1;

                    Serial.println(
                        "-> MOVING_1"
                    );

                    break;


                case MOVING_1:

                    currentState =
                        HOVER_2;

                    Serial.println(
                        "-> HOVER_2"
                    );

                    break;


                case HOVER_2:

                    currentState =
                        MOVING_2;

                    Serial.println(
                        "-> MOVING_2"
                    );

                    break;


                case MOVING_2:

                    currentState =
                        HOVER_3;

                    Serial.println(
                        "-> HOVER_3"
                    );

                    break;


                case HOVER_3:

                    currentState =
                        LANDING;

                    Serial.println(
                        "-> LANDING"
                    );

                    break;


                case LANDING:

                    // ========================================
                    // IMPORTANTE:
                    // NÃO VOLTA PARA TAKEOFF
                    // ========================================

                    currentState =
                        MISSION_FINISHED;

                    Serial.println(
                        "-> MISSION_FINISHED"
                    );

                    break;


                case MISSION_FINISHED:

                    break;
            }
        }
    }


    // ========================================================
    // 4. DEFINE COMANDO
    // ========================================================

    switch (currentState)
    {

        // ====================================================
        // TAKEOFF
        // ====================================================

        case TAKEOFF:

            current_roll = 0.0;

            current_pitch = 0.0;

            current_yaw = 0.0;

            current_thrust = 45000;

            break;


        // ====================================================
        // HOVER
        // ====================================================

        case HOVER_1:

        case HOVER_2:

        case HOVER_3:

            current_roll = 0.0;

            current_pitch = 0.0;

            current_yaw = 0.0;

            current_thrust = 32767;

            break;


        // ====================================================
        // MOVIMENTO
        // ====================================================

        case MOVING_1:

        case MOVING_2:

            current_roll = 0.0;

            current_pitch = -15.0;

            current_yaw = 0.0;

            current_thrust = 32767;

            break;


        // ====================================================
        // LANDING
        // ====================================================

        case LANDING:

            current_roll = 0.0;

            current_pitch = 0.0;

            current_yaw = 0.0;


            // ------------------------------------------------
            // Primeiros 5 segundos:
            // reduz thrust progressivamente
            // ------------------------------------------------

            if (
                timeInState < 5000
            )
            {
                float progress =
                    (float)timeInState /
                    5000.0;


                float reverse_progress =
                    1.0 - progress;


                const uint16_t
                    MIN_LIFT_THRUST =
                    18000;


                const uint16_t
                    MAX_HOVER_THRUST =
                    32767;


                current_thrust =
                    MIN_LIFT_THRUST +
                    (
                        (
                            MAX_HOVER_THRUST -
                            MIN_LIFT_THRUST
                        )
                        *
                        (
                            reverse_progress *
                            reverse_progress
                        )
                    );
            }

            // ------------------------------------------------
            // Depois de 5 segundos:
            // motor desligado
            // ------------------------------------------------

            else
            {
                current_thrust = 0;
            }

            break;


        // ====================================================
        // MISSÃO FINALIZADA
        // ====================================================

        case MISSION_FINISHED:

            current_roll = 0.0;

            current_pitch = 0.0;

            current_yaw = 0.0;

            current_thrust = 0;

            break;
    }


    // ========================================================
    // 5. ENVIA A 50 Hz
    // ========================================================

    if (
        currentMillis -
        lastCommandTime >=
        COMMAND_INTERVAL
    )
    {
        lastCommandTime =
            currentMillis;


        send_crtp_command(
            current_roll,
            current_pitch,
            current_yaw,
            current_thrust
        );


        // ----------------------------------------------------
        // Debug
        // ----------------------------------------------------

        static unsigned long
            lastPrint = 0;


        if (
            currentMillis -
            lastPrint >= 500
        )
        {
            lastPrint =
                currentMillis;


            Serial.print(
                "Estado: "
            );


            switch (currentState)
            {

                case TAKEOFF:
                    Serial.print(
                        "TAKEOFF"
                    );
                    break;

                case HOVER_1:
                    Serial.print(
                        "HOVER_1"
                    );
                    break;

                case MOVING_1:
                    Serial.print(
                        "MOVING_1"
                    );
                    break;

                case HOVER_2:
                    Serial.print(
                        "HOVER_2"
                    );
                    break;

                case MOVING_2:
                    Serial.print(
                        "MOVING_2"
                    );
                    break;

                case HOVER_3:
                    Serial.print(
                        "HOVER_3"
                    );
                    break;

                case LANDING:
                    Serial.print(
                        "LANDING"
                    );
                    break;

                case MISSION_FINISHED:
                    Serial.print(
                        "MISSION_FINISHED"
                    );
                    break;
            }


            Serial.print(
                " | Thrust: "
            );

            Serial.println(
                current_thrust
            );
        }
    }
}