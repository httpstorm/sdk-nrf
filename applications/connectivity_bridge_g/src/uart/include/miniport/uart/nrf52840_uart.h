// ***************************************************************************
//
// File:          nrf52840_uart.h
// Author(s):     Georgi Valkov (GV)
// Project:       UART miniport driver for nrf52840
// Description:   hadrware specific definitions and types
//
// Copyright (C) 2020-2023 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
// ***************************************************************************

#ifndef NRF52840_UART_H
#define NRF52840_UART_H


// Include files *************************************************************

#ifdef RX
#undef RX
#endif

#include <os_layer.h>

#if CONFIG_BOARD_NRF9160DK_NRF9160_NS || CONFIG_BOARD_NRF9160DK_NRF9160_S || CONFIG_BOARD_THINGY91_NRF9160
#include <arch/nordic/nrf9160.h>
#include <arch/nordic/nrf9160_bitfields.h>
#else
#include <arch/nordic/nrf52840.h>
#include <arch/nordic/nrf52840_bitfields.h>
#endif


// Macros ********************************************************************

#ifndef NRF52840_USE_DMA
#define NRF52840_USE_DMA 1
#endif // !NRF52840_USE_DMA

#ifndef NRF52840_FRAMING_ERROR_DELAY
#define NRF52840_FRAMING_ERROR_DELAY 3000000
#endif

// array size
#define NRF52840_UART_COUNT(a) (sizeof((a)) / sizeof(*(a)))

#if CONFIG_BOARD_NRF9160DK_NRF9160_NS || CONFIG_BOARD_NRF9160DK_NRF9160_S
#define NRF52840_UARTE_COUNT 4
#else
#define NRF52840_UARTE_COUNT 2
#endif

// RXD.AMMOUNT is updated only at the end of each DMA transfer.
// This makes it hard to keep an accurate track of the number of characters
// received in the middle of each DMA transfer. The best we can do is count
// the number of EVENTS_RXDRDY in the interrupt handler, but these are
// easy to miss. Use a smaller transfer size, to remain in sync with DMA.
#define NRF52840_UART_ERRATA_MAX_COUNT_HS 2048
#define NRF52840_UART_ERRATA_MAX_COUNT_RX 256
#define NRF52840_UART_ERRATA_MAX_COUNT_LL 16
//? #define NRF52840_UART_ERRATA_MAX_COUNT_LL 4
#define NRF52840_UART_ERRATA_RECV_JOB_RX 0

#ifndef NRF52840_UART_BUFFER_SIZE
#define NRF52840_UART_BUFFER_SIZE 4096
#endif // !NRF52840_UART_BUFFER_SIZE

#define NRF52840_UART_BUFFER_MASK (NRF52840_UART_BUFFER_SIZE - 1)

#if (NRF52840_UART_BUFFER_SIZE < 2) || (NRF52840_UART_BUFFER_SIZE & NRF52840_UART_BUFFER_MASK)
#error NRF52840_UART_BUFFER_SIZE must be power of 2
#endif

#define NRF52840_UART_INT_RX (UARTE_INTENSET_ENDRX_Msk | UARTE_INTENSET_RXDRDY_Msk)
#define NRF52840_UART_INT_TX (UARTE_INTENSET_ENDTX_Msk | UARTE_INTENSET_TXDRDY_Msk)
#define NRF52840_UART_INT_ENABLE(reg, mask) (reg->INTENSET = mask)
#define NRF52840_UART_INT_DISABLE_RX(reg) reg->INTENSET & NRF52840_UART_INT_RX; reg->INTENCLR = NRF52840_UART_INT_RX
#define NRF52840_UART_INT_DISABLE_TX(reg) reg->INTENSET & NRF52840_UART_INT_TX; reg->INTENCLR = NRF52840_UART_INT_TX
#define NRF52840_UART_INT_DISABLE_ALL(reg) reg->INTENSET; reg->INTENCLR = 0xffffffff
#define NRF52840_UART_INT_ENABLED_RX(reg) (reg->INTENSET & NRF52840_UART_INT_RX)
#define NRF52840_UART_INT_ENABLED_TX(reg) (reg->INTENSET & NRF52840_UART_INT_TX)


// Types *********************************************************************

// prototype of uart API type
typedef struct uart_api uart_api;
typedef enum UART_STATUS_E UART_STATUS_E;

typedef struct nrf52840_uart_state
{
	// RX and TX indexes for panic UART
	// intentionally put in the beginning of the structure to aid debugging
	uint32_t rx_a;
	uint32_t rx_b;
	uint32_t rx_B;
	uint32_t tx_A;
	uint32_t tx_a;
	uint32_t tx_b;

	uint32_t tx_ready;
	uint32_t loopback;

	union
	{
#ifdef NRF52840_H
		NRF_UART_Type * reg;
#endif
		NRF_UARTE_Type * reg_dma;
	};

	union
	{
#ifdef NRF52840_H
		UART_PSEL_Type pin_select;
#endif
		UARTE_PSEL_Type pin_select_dma;
	};

#if NRF52840_UART_ERRATA_MAX_COUNT_RX
	uint32_t max_rx;
#endif

	// RX and TX buffers for panic UART
	uint32_t size;
	uint32_t mask;
	char rx[NRF52840_UART_BUFFER_SIZE];
	char tx[NRF52840_UART_BUFFER_SIZE];
} nrf52840_uart_state;

// the hardware library defines this
// as pointer type to uart base address
// the generic code uses void *
#define uart_reg nrf52840_uart_state *

typedef struct baudrate_map_t
{
	uint32_t baudrate;
	uint32_t baud_reg;
} baudrate_map_t;

enum NRF52840_UART_PSEL
{
	NRF52840_UART_PSEL_PIN = 0x1f,	// 0-31
	NRF52840_UART_PSEL_PORT = 0x20,	// 32
	NRF52840_UART_PSEL_MASK = NRF52840_UART_PSEL_PIN | NRF52840_UART_PSEL_PORT, // 0-64
	NRF52840_UART_PSEL_DISCONNECT = 0x80000000,
};


// Local Prototypes **********************************************************

// Global Prototypes *********************************************************

// Function prototypes *******************************************************

_BEGIN_CPLUSPLUS

#if defined(__APPLE__) ||  defined(__ZEPHYR__)
#define __attribute__(x)
#endif

// hardware interface

#if CONFIG_PINCTRL
int nrf52840_uart_pinctrl(uart_api * uart);
#endif

void nrf52840_uart_abort(uart_api * uart);
UART_STATUS_E nrf52840_uart_framing_error_check(uart_api * uart, uint8_t random_delay);
UART_STATUS_E nrf52840_uart_set_duplex(uart_api * uart, int enable);
UART_STATUS_E nrf52840_uart_set_baud_rate(uart_api * uart, uint32_t baud, uint32_t bus);
UART_STATUS_E nrf52840_uart_recv_job(uart_api * uart, char * msg, size_t bytes_requested);

#if NRF52840_USE_DMA
UART_STATUS_E nrf52840_uart_init_dma(uart_api * uart);
void nrf52840_uart_deinit_dma(uart_api * uart);
UART_STATUS_E nrf52840_uart_configure_dma(uart_api * uart);
void nrf52840_uart_start_rx_dma(uart_api * uart);
UART_STATUS_E nrf52840_uart_send_byte_dma(uart_api * uart, char c);
UART_STATUS_E nrf52840_uart_recv_byte_dma(uart_api * uart, char * c);
UART_STATUS_E nrf52840_uart_send_job_dma(uart_api * uart);
int nrf52840_uart_handler_int_dma(uart_api * uart, int int_state);
void __attribute__((interrupt)) nrf52840_uart_handler_raw_dma_0(void);
void __attribute__((interrupt)) nrf52840_uart_handler_raw_dma_1(void);
#if CONFIG_BOARD_NRF9160DK_NRF9160_NS || CONFIG_BOARD_NRF9160DK_NRF9160_S
void __attribute__((interrupt)) nrf52840_uart_handler_raw_dma_2(void);
void __attribute__((interrupt)) nrf52840_uart_handler_raw_dma_3(void);
#endif
#else
UART_STATUS_E nrf52840_uart_init(uart_api * uart);
void nrf52840_uart_deinit(uart_api * uart);
UART_STATUS_E nrf52840_uart_configure(uart_api * uart);
UART_STATUS_E nrf52840_uart_send_byte(uart_api * uart, char c);
UART_STATUS_E nrf52840_uart_send_byte_sync(uart_api * uart, char c);
UART_STATUS_E nrf52840_uart_recv_byte(uart_api * uart, char * c);
UART_STATUS_E nrf52840_uart_send_job(uart_api * uart);
int nrf52840_uart_handler_int(uart_api * uart, int int_state);
void __attribute__((interrupt)) nrf52840_uart_handler_raw_0(void);
#endif

#if defined(__APPLE__) ||  defined(__ZEPHYR__)
#undef __attribute__
#endif

_END_CPLUSPLUS


// Global variables **********************************************************

// End of file ***************************************************************

#endif // NRF52840_UART_H
