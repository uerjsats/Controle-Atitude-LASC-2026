//adicionado por missão
#ifndef CRTP_SERIAL_LINK_H_
#define CRTP_SERIAL_LINK_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa o link serial CRTP.
 * Configura o periférico UART, aloca buffers DMA e instancia a task do FreeRTOS
 * responsável pela recepção, validação de checksum e roteamento das portas do protocolo.
 */
void crtpSerialLinkInit(void);
void crtpSerialLinkSendReadySignal(void);

#ifdef __cplusplus
}
#endif

#endif /* CRTP_SERIAL_LINK_H_ */