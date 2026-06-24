//Adicionado por missão
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "crtp.h"
#include "commander.h"
#include "estimator.h"
#include "stabilizer_types.h"
//ALterado por missão
#include "crtp_serial_link.h"

static const char *TAG = "CRTP_SERIAL_LINK";

#define CRTP_UART_NUM      UART_NUM_1
#define CRTP_TX_PIN        43       
#define CRTP_RX_PIN        44       
#define CRTP_BAUDRATE      115200   
#define UART_BUF_SIZE      1024

#define CRTP_START_BYTE    0xAA     

typedef struct {
    uint8_t header;
    uint8_t size;
    uint8_t data[CRTP_MAX_DATA_SIZE];
    uint8_t checksum;
} CrtpRxPacket_t;

typedef enum { 
    STATE_WAIT_START, 
    STATE_HEADER, 
    STATE_SIZE, 
    STATE_DATA, 
    STATE_CHECKSUM 
} CrtpRxState_t;

static CrtpRxState_t rx_state = STATE_WAIT_START;
static CrtpRxPacket_t rx_packet;
static uint8_t data_idx = 0;
static uint8_t calculated_checksum = 0;

static void send_crtp_ack(uint8_t header) {
    uint8_t ack_packet[5];
    ack_packet[0] = CRTP_START_BYTE; 
    ack_packet[1] = CRTP_START_BYTE;
    ack_packet[2] = header;          
    ack_packet[3] = 0x00;
    ack_packet[4] = header + 0x00;
    uart_write_bytes(CRTP_UART_NUM, (const char*)ack_packet, sizeof(ack_packet));
}

static void process_crtp_byte(uint8_t byte) {
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
                
                if (port == 3) {
                    setpoint_t serial_setpoint = {0};
                    
                    // Preenchimento imperativo do Setpoint Serial
                    serial_setpoint.mode.x = modeDisable;
                    serial_setpoint.mode.y = modeDisable;
                    serial_setpoint.mode.z = modeDisable;
                    serial_setpoint.mode.roll = modeAbs;
                    serial_setpoint.mode.pitch = modeAbs;
                    serial_setpoint.mode.yaw = modeVelocity; 
                    serial_setpoint.mode.quat = modeDisable;

                    memcpy(&serial_setpoint.attitude.roll, &rx_packet.data[0], sizeof(float));
                    memcpy(&serial_setpoint.attitude.pitch, &rx_packet.data[4], sizeof(float));
                    memcpy(&serial_setpoint.attitudeRate.yaw, &rx_packet.data[8], sizeof(float));
                    
                    uint16_t thrust_raw = 0;
                    memcpy(&thrust_raw, &rx_packet.data[12], sizeof(uint16_t));
                    serial_setpoint.thrust = thrust_raw;
                    
                    ESP_LOGI(TAG, "RX CMD | Roll: %.2f | Pitch: %.2f | YawRate: %.2f | Thrust: %.0f", 
                        serial_setpoint.attitude.roll, 
                        serial_setpoint.attitude.pitch, 
                        serial_setpoint.attitudeRate.yaw, 
                        serial_setpoint.thrust);
                    
                    // Submete o pacote ao core. O commander.c ignorará em background se o rádio estiver ativo.
                    commanderSetSetpoint(&serial_setpoint, COMMANDER_PRIORITY_SERIAL);
                } else {
                    CRTPPacket internal_packet;
                    internal_packet.port = port;
                    internal_packet.channel = rx_packet.header & 0x03; // Extrai o canal dos bits inferiores
                    internal_packet.size = rx_packet.size;
                    memcpy(internal_packet.data, rx_packet.data, rx_packet.size);
                    
                    // CORREÇÃO: Passando a porta (cast implícito/explícito para CRTPPort) e o ponteiro
                    crtpReceivePacketBlock((CRTPPort)port, &internal_packet);
                    ESP_LOGD(TAG, "Pacote CRTP despachado com sucesso. Porta: %d", port);
                }
                send_crtp_ack(rx_packet.header);
            } else {
                ESP_LOGE(TAG, "Erro: Falha de Checksum no pacote serial!");
            }
            
            rx_state = STATE_WAIT_START;
            break;
    }
}

static void crtp_serial_link_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Falha ao alocar buffer da UART");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Link Serial CRTP iniciado. Aguardando pacotes...");
    while (1) {
        int len = uart_read_bytes(CRTP_UART_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        for (int i = 0; i < len; i++) {
            process_crtp_byte(data[i]);
        }
    }
}

void crtpSerialLinkInit(void) {
    uart_config_t uart_config = {
        .baud_rate = CRTP_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(CRTP_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CRTP_UART_NUM, CRTP_TX_PIN, CRTP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(CRTP_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    
    xTaskCreatePinnedToCore(crtp_serial_link_task, "crtpSerialLink", 4096, NULL, 5, NULL, 1);
    
}

void crtp_send_ready_to_fly(void) {

    uint8_t header = 0xF0; 
    uint8_t size = 1;       // 1 byte de carga útil (payload)
    uint8_t payload = 0x01; // 0x01 é o código arbitrário que representa "Ready to Fly"
    
    // Checksum baseado na lógica interna: Header + Size + Soma dos dados
    uint8_t checksum = header + size + payload;

    // Buffer de 6 bytes totais
    uint8_t tx_packet[6];
    tx_packet[0] = CRTP_START_BYTE; // 0xAA
    tx_packet[1] = CRTP_START_BYTE; // 0xAA 
    tx_packet[2] = header;          // 0xF0
    tx_packet[3] = size;            // 0x01
    tx_packet[4] = payload;         // 0x01
    tx_packet[5] = checksum;        

    uart_write_bytes(CRTP_UART_NUM, (const char*)tx_packet, sizeof(tx_packet));
    ESP_LOGI(TAG, "Sinal 'Ready to Fly' enviado via TX.");
}