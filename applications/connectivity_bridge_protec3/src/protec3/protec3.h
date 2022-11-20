#ifndef GUARD_PROTEC3_H
#define GUARD_PROTEC3_H


#if CONFIG_BOARD_PROTEC3_NRF52840
#include "../gpio/gpio_t.h"
#endif

#include "../uart/include/uart_miniport.h"


enum UART_LINK_CMD_E
{
	UART_LINK_CMD_INVALID = -1,
	UART_LINK_CMD_DOWN = 0,
	UART_LINK_CMD_UP = 1,
	UART_LINK_CMD_QUERY = 2,
};


extern int uart_forward;


/* retrieve the MAC address of the currently connected phone
 * if currently connected addr will contain MAC address in format
 *  001122334455
 * and return value will be true
 * If no device is connected return value will be false and addr is undefined
 */
bool ble_get_mac(char addr[13]);

// process UART data
int uart_process(const char * msg, int count);

// UART send data
int uart_tx_enqueue_ext(uint8_t * data, size_t data_len, uint8_t dev_idx);


#endif /* GUARD_PROTEC3_H */
