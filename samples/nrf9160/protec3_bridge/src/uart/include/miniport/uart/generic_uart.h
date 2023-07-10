// ***************************************************************************
//
// File:         generic_uart.h
// Author(s):    Georgi Valkov (GV)
// Project:      generic UART miniport driver
// Description:  high-level API definitions and types
//
// Copyright (C) 2019-2023 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
// ***************************************************************************

#ifndef GENERIC_UART_H
#define GENERIC_UART_H

// Include files *************************************************************

#include <os_layer.h>


// Macros ********************************************************************

// Types *********************************************************************

// uart status
typedef enum UART_STATUS_E
{
	UART_STATUS_FRAMING = -2,
	UART_STATUS_FAIL = FAIL,
	UART_STATUS_OK = OK,
} UART_STATUS_E;

// uart parity mode
enum UART_PARITY_E
{
	UART_PARITY_ENVALID = -1,

#ifndef UART_PARITY_NONE
	UART_PARITY_NONE = 0,
#endif

#ifndef UART_PAL_H
#ifndef UART_PARITY_ODD
	UART_PARITY_ODD = 1,
#endif

#ifndef UART_PARITY_EVEN
	UART_PARITY_EVEN = 2,
#endif
#endif
};

// uart word size
enum UART_WORD_SIZE_E
{
	UART_WORD_SIZE_8 = 0,
	UART_WORD_SIZE_9 = 1,
	UART_WORD_SIZE_10 = 2,
	UART_WORD_SIZE_7 = 3,
};

// uart command 0: write, 1: read
enum UART_CMD_E
{
	UART_CMD_WRITE = 0,
	UART_CMD_READ = 1,
};

typedef union
{
	uint8_t value;                  // 0xff

	struct
	{
		uint8_t wait_rx : 1;        // 0x01
		uint8_t wait_tx : 1;        // 0x02
		uint8_t stop : 1;           // 0x04
		uint8_t tx : 1;             // 0x08
		uint8_t : 2;                // 0x30
		uint8_t framing_err : 1;    // 0x40
		uint8_t valid : 1;          // 0x80
	};
} uart_status;

typedef union
{
	uint32_t value;                 // 0x ffff ffff

	struct
	{
		uint32_t int_priority : 4;  // 0x 0000 000f  interrupt priority
		uint32_t use_interrupts : 1;// 0x 0000 0010  enable interrupts
		uint32_t use_dma : 1;       // 0x 0000 0020  enable DMA
		uint32_t idle_ends_recv : 1;// 0x 0000 0040  allow recv to return less then requested bytes if the RX line becomes idle
		uint32_t duplex : 1;        // 0x 0000 0080  enable full duplex mode
		uint32_t parity : 2;        // 0x 0000 0300  UART_PARITY_NONE, UART_PARITY_ODD, UART_PARITY_EVEN
		uint32_t word_size : 2;     // 0x 0000 0c00  UART_WORD_SIZE_8, UART_WORD_SIZE_7 - UART_WORD_SIZE_10
		uint32_t stop_bits_2 : 1;   // 0x 0000 1000  0: 1 stop bit, 1: 2 stop bits
		uint32_t msb_first : 1;     // 0x 0000 2000  0: LSB first, 1: MSB first
		uint32_t inv_rx : 1;        // 0x 0000 4000  invert RX line polarity
		uint32_t inv_tx : 1;        // 0x 0000 8000  invert TX line polarity
		uint32_t flow_control : 1;  // 0x 0001 0000  flow control
		uint32_t : 14;              // 0x 7ffe 0000  reserved
		uint32_t enabled : 1;       // 0x 8000 0000  enabled
	};
} uart_config;

#define UART_OWN_ADDRESS_T \
	union\
	{\
		uint32_t address;\
		\
		struct\
		{\
			uint16_t address1;\
			uint16_t address2;\
		};\
	}

// the hardware library defines this
// as pointer type to UART base address
// the generic code uses void *
#ifndef uart_reg
#define uart_reg void *
#endif

#ifndef uart_task
#define uart_task size_t
#endif


// Types *********************************************************************

typedef struct uart_api uart_api;

typedef UART_STATUS_E(*fn_uart_init)(uart_api * uart);
typedef void(*fn_uart_deinit)(uart_api * uart);
typedef void(*fn_uart_abort)(uart_api * uart);
typedef UART_STATUS_E(*fn_uart_set_duplex)(uart_api * uart, int enable);
typedef UART_STATUS_E(*fn_uart_set_baud_rate)(uart_api * uart, uint32_t baud, uint32_t bus);
typedef int(*fn_uart_handler_int)(uart_api * uart, int state);
typedef UART_STATUS_E(*fn_uart_send_byte)(uart_api * uart, char c);
typedef UART_STATUS_E(*fn_uart_recv_byte)(uart_api * uart, char * c);
typedef UART_STATUS_E(*fn_uart_send_job)(uart_api * uart);
typedef UART_STATUS_E(*fn_uart_recv_job)(uart_api * uart, char * msg, size_t bytes_requested);
typedef ssize_t(*fn_uart_send)(uart_api * uart, const char * msg, size_t count);
typedef ssize_t(*fn_uart_recv)(uart_api * uart, char * msg, size_t count);
typedef int (*fn_uart_wake_tx)(uart_api * uart);
typedef int (*fn_uart_wake_rx)(uart_api * uart);

#ifndef UART_HANDLER_TYPE_DEFINED
typedef int(*fn_uart_handler_send)(uart_api * uart);
typedef int(*fn_uart_handler_recv)(uart_api * uart);
#endif

struct uart_job_rx
{
	char * msg;
	size_t bytes_requested;
	size_t bytes_transferred;
};

struct uart_job_tx
{
	const char * msg;
	size_t bytes_requested;
	size_t bytes_transferred;
};

typedef struct uart_api
{
	// interface index and base address
	size_t index;
	uart_reg reg;
	uart_task task_rx;
	uart_task task_tx;
	int int_number;
	void * int_prolog;
	uart_status status;
	uart_config config;
	uint32_t baud_rate;

	UART_OWN_ADDRESS_T;

	struct uart_job_rx job_rx;
	struct uart_job_tx job_tx;

#if NANO_RTOS
	nano_mutex_t mutex_tx;
	nano_mutex_t mutex_rx;
#endif

#if __ZEPHYR__
	struct k_event event_tx;
	struct k_event event_rx;
#endif

	// interface functions
	fn_uart_init init;
	fn_uart_deinit deinit;
	fn_uart_abort abort;
	fn_uart_set_baud_rate set_baud_rate;
	fn_uart_handler_int handler_int;
	fn_uart_handler_send handler_send;
	fn_uart_handler_recv handler_recv;
	fn_uart_set_duplex set_duplex;
	fn_uart_send_byte send_byte;
	fn_uart_recv_byte recv_byte;
	fn_uart_send_job send_job;
	fn_uart_recv_job recv_job;
	fn_uart_send send;
	fn_uart_recv recv;
	fn_uart_wake_tx uart_wake_tx;
	fn_uart_wake_rx uart_wake_rx;
} uart_api;


// Local Prototypes **********************************************************

// Global Prototypes *********************************************************

// Function prototypes *******************************************************

_BEGIN_CPLUSPLUS

ssize_t uart_send(uart_api * uart, const char * msg, size_t count);
ssize_t uart_recv(uart_api * uart, char * msg, size_t count);

int uart_wake_tx(uart_api * uart);
int uart_wake_rx(uart_api * uart);

// constructor for uart_api, provided in the hardware miniport interface
int uart_create(uart_api * uart, size_t interface_index);
uart_api * uart_open(size_t interface_index);

_END_CPLUSPLUS


// Global variables **********************************************************

// End of file ***************************************************************

#endif // GENERIC_UART_H
