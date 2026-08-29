#include <Arduino.h> 

// ---------------------------------------------------------
// Definições de Hardware e UART
// ---------------------------------------------------------
#define TX_PIN        43       
#define RX_PIN        44      
#define BAUDRATE      115200   

#define CRTP_START_BYTE  0xAA
#define COMMANDER_HEADER 0x30 

// ---------------------------------------------------------
// Estrutura de Dados do Movimento (Payload)
// ---------------------------------------------------------
struct CommanderPayload {
    float roll;
    float pitch;    
    float yaw;       
    uint16_t thrust; 
} __attribute__((packed)); 

// ---------------------------------------------------------
// Variáveis de Frequência, Tempo e Sincronização
// ---------------------------------------------------------
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_INTERVAL = 20; // 50Hz (20ms)

unsigned long stateStartTime = 0;
const unsigned long STATE_DURATION = 10000;

// Flag de bloqueio para o sinal "Ready to Fly"
bool is_ready_to_fly = false;

// ---------------------------------------------------------
// Máquina de Estados de Voo
// ---------------------------------------------------------
enum FlightState {
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

// ---------------------------------------------------------
// Variáveis atuais a serem enviadas
// ---------------------------------------------------------
float current_roll = 0.0;
float current_pitch = 0.0;
float current_yaw = 0.0;
uint16_t current_thrust = 0;

// ---------------------------------------------------------
// Função de Transmissão CRTP (Setpoint)
// ---------------------------------------------------------
void send_crtp_command(
    float roll,
    float pitch,
    float yaw,
    uint16_t thrust
) {
    
    CommanderPayload payload;
    
    payload.roll = roll;      
    payload.pitch = pitch;    
    payload.yaw = yaw;        
    payload.thrust = thrust;  

    uint8_t payload_size = sizeof(payload); 
    uint8_t checksum = 0;
    
    Serial1.write(CRTP_START_BYTE); 
    Serial1.write(CRTP_START_BYTE); 

    Serial1.write(COMMANDER_HEADER);
    checksum += COMMANDER_HEADER;    

    Serial1.write(payload_size);    
    checksum += payload_size;       

    uint8_t* payload_bytes = (uint8_t*)&payload;
    
    for (int i = 0; i < payload_size; i++) {    
        Serial1.write(payload_bytes[i]);
        checksum += payload_bytes[i];            
    }

    Serial1.write(checksum); 
}

// ---------------------------------------------------------
// Função de Recepção CRTP (Parser Não Bloqueante)
// ---------------------------------------------------------
void process_rx_ready_to_fly() {

    static uint8_t rx_state = 0;
    static uint8_t calculated_checksum = 0;

    // Processa todos os bytes disponíveis no buffer sem travar o loop
    while (Serial1.available() > 0 && !is_ready_to_fly) {

        uint8_t c = Serial1.read();
        
        switch (rx_state) {

            case 0:
                // Aguarda o primeiro START_BYTE
                if (c == CRTP_START_BYTE)
                    rx_state = 1;

                break;


            case 1:
                // Aguarda o segundo START_BYTE
                if (c == CRTP_START_BYTE)
                    rx_state = 2;
                else
                    rx_state = 0;

                break;


            case 2:
                // Aguarda HEADER 0xF0
                if (c == 0xF0) {

                    calculated_checksum = c;
                    rx_state = 3;

                } else {

                    rx_state = 0;
                }

                break;


            case 3:
                // Aguarda SIZE = 0x01
                if (c == 0x01) {

                    calculated_checksum += c;
                    rx_state = 4;

                } else {

                    rx_state = 0;
                }

                break;


            case 4:
                // Aguarda PAYLOAD = 0x01
                if (c == 0x01) {

                    calculated_checksum += c;
                    rx_state = 5;

                } else {

                    rx_state = 0;
                }

                break;


            case 5:
                // Aguarda e valida CHECKSUM
                if (c == calculated_checksum) {

                    is_ready_to_fly = true;

                    Serial.println();
                    Serial.println(
                        ">>> SINAL [READY TO FLY] RECEBIDO! <<<"
                    );

                    Serial.println(
                        "=== INICIANDO MISSAO CRTP ==="
                    );

                    Serial.println(
                        "-> TAKEOFF"
                    );

                    delay(2000);

                    // Sincroniza os cronômetros
                    stateStartTime = millis();
                    lastCommandTime = millis();

                } else {

                    Serial.println(
                        "Erro: Checksum invalido no sinal RTF."
                    );
                }

                rx_state = 0;

                break;
        }
    }
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {

    Serial.begin(115200); 

    delay(1000);

    Serial.println(
        "=== AGUARDANDO SINAL DE SINCRONIZACAO (READY TO FLY) ==="
    );

    Serial1.begin(
        BAUDRATE,
        SERIAL_8N1,
        RX_PIN,
        TX_PIN
    );
}

// ---------------------------------------------------------
// LOOP - Sequência Contínua Sem Bloqueios
// ---------------------------------------------------------
void loop() {

    // =====================================================
    // 1. ESPERA PELO READY TO FLY
    // =====================================================

    if (!is_ready_to_fly) {

        process_rx_ready_to_fly();

        return;
    }


    // =====================================================
    // 2. LÓGICA DA MÁQUINA DE ESTADOS
    // =====================================================

    unsigned long currentMillis = millis();

    unsigned long timeInState =
        currentMillis - stateStartTime;


    // =====================================================
    // 3. MUDANÇA DE ESTADO
    // =====================================================

    if (
        currentState != MISSION_FINISHED &&
        timeInState >= STATE_DURATION
    ) {

        stateStartTime = currentMillis;

        timeInState = 0;


        switch(currentState) {

            case TAKEOFF:

                currentState = HOVER_1;

                Serial.println(
                    "-> HOVER_1"
                );

                break;


            case HOVER_1:

                currentState = MOVING_1;

                Serial.println(
                    "-> MOVING_1"
                );

                break;


            case MOVING_1:

                currentState = HOVER_2;

                Serial.println(
                    "-> HOVER_2"
                );

                break;


            case HOVER_2:

                currentState = MOVING_2;

                Serial.println(
                    "-> MOVING_2"
                );

                break;


            case MOVING_2:

                currentState = HOVER_3;

                Serial.println(
                    "-> HOVER_3"
                );

                break;


            case HOVER_3:

                currentState = LANDING;

                Serial.println(
                    "-> LANDING"
                );

                break;


            case LANDING:

                // ------------------------------------------------
                // ALTERAÇÃO PRINCIPAL:
                // A missão termina aqui.
                // Não volta para TAKEOFF.
                // ------------------------------------------------

                currentState = MISSION_FINISHED;

                Serial.println(
                    "-> MISSION_FINISHED"
                );

                break;


            case MISSION_FINISHED:

                break;
        }
    }


    // =====================================================
    // 4. DEFINE OS VALORES DOS COMANDOS
    // =====================================================

    switch(currentState) {

        // -------------------------------------------------
        // TAKEOFF
        // -------------------------------------------------

        case TAKEOFF:

            current_roll = 0.0;

            current_pitch = 0.0;

            current_yaw = 0.0;

            current_thrust = 45000;

            break;


        // -------------------------------------------------
        // HOVER
        // -------------------------------------------------

        case HOVER_1:

        case HOVER_2:

        case HOVER_3:

            current_roll = 0.0;

            current_pitch = 0.0;

            current_yaw = 0.0;

            current_thrust = 32767;

            break;


        // -------------------------------------------------
        // MOVIMENTO
        // -------------------------------------------------

        case MOVING_1:

        case MOVING_2:

            current_roll = 0.0;

            current_pitch = -15.0;

            current_yaw = 0.0;

            current_thrust = 32767;

            break;


        // -------------------------------------------------
        // LANDING
        // -------------------------------------------------

        case LANDING:

            current_roll = 0.0;

            current_pitch = 0.0;

            current_yaw = 0.0;


            if (timeInState < 5000) {

                float progress =
                    (float)timeInState / 5000.0;


                float reverse_progress =
                    1.0 - progress;


                const uint16_t MIN_LIFT_THRUST =
                    18000;


                const uint16_t MAX_HOVER_THRUST =
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

            } else {

                current_thrust = 0;
            }

            break;


        // -------------------------------------------------
        // MISSÃO FINALIZADA
        // -------------------------------------------------

        case MISSION_FINISHED:

            current_roll = 0.0;

            current_pitch = 0.0;

            current_yaw = 0.0;

            current_thrust = 0;

            break;
    }


    // =====================================================
    // 5. ENVIA O COMANDO A 50 Hz
    // =====================================================

    if (
        currentMillis - lastCommandTime >=
        COMMAND_INTERVAL
    ) {

        lastCommandTime = currentMillis;


        send_crtp_command(
            current_roll,
            current_pitch,
            current_yaw,
            current_thrust
        );


        // Mantém exatamente a lógica do seu código original
        while (Serial1.available()) {
            Serial1.read();
        }
    }
}
