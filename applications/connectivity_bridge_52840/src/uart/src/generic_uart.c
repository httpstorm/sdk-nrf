//****************************************************************************
//
// File:         generic_uart.c
// Author(s):    Georgi Valkov (GV)
// Project:      generic UART miniport driver
// Description:  high-level API
//
// Copyright (C) 2019 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
//****************************************************************************

// Include files *************************************************************

#if defined (EUROS) || defined (OSEK)
#include <int.h>
#endif

#include <miniport/uart/generic_uart.h>


// Macro definitions *********************************************************

// Types *********************************************************************

// Local Prototypes **********************************************************

// Global variables **********************************************************

// Functions *****************************************************************

// send count bytes from buffer msg over uart interface
// returns number of bytes sent
// stops if errors are detected
size_t uart_send(struct uart_api * uart, const char * msg, size_t count)
{
	if (!msg)
	{
		return FAIL;
	}

	if (!count)
	{
		return 0;
	}

	int_state_t int_state = int_unavailable();

#ifdef NANO_RTOS
	// we are allowed to continue on success,
	// when nano RTOS is not running or called from an interrupt handler
	// or interrupts are disabled
	// in any other case, abort the call
	if (mutex_lock(&uart->mutex_tx, NANO_TIMEOUT_WAIT) < NANO_MUTEX_FAIL)
	{
		return 0;
	}
#endif

	if (uart->config.use_interrupts && uart->send_job && !int_state)
	{
		// job transfer is more efficient then byte transfer
		if (FAIL != task_id_get())
		{
			// the job might begin processing as soon as msg is set
			uart->job_tx.bytes_transferred = 0;
			uart->job_tx.bytes_requested = count;
			uart->job_tx.msg = msg;

			if (OK == uart->send_job(uart))
			{
				count = uart->job_tx.bytes_transferred;

#ifdef NANO_RTOS
				mutex_unlock(&uart->mutex_tx);
#endif

				return count;
			}
		}
	}

	// byte transfer
	size_t c = 0;

	for (; (c < count) && !uart->status.stop; c++)
	{
		if (OK != uart->send_byte(uart, msg[c]))
		{
			break;
		}
	}

#ifdef NANO_RTOS
	mutex_unlock(&uart->mutex_tx);
#endif

	return c;
}

// receives count bytes to buffer msg over uart interface
// returns number of bytes received or FAIL if invalid parameter is specified
// may return less than count bytes, if idle_ends_recv is enabled,
// and the RX line becomes idle after receiving at least one byte
// stops if errors are detected
size_t uart_recv(struct uart_api * uart, char * msg, size_t count)
{
	if (!msg)
	{
		return FAIL;
	}

	if (!count)
	{
		return 0;
	}

	int_state_t int_state = int_unavailable();

#ifdef NANO_RTOS
	// we are allowed to continue on success,
	// when nano RTOS is not running or called from an interrupt handler
	// or interrupts are disabled
	// in any other case, abort the call
	if (mutex_lock(&uart->mutex_rx, NANO_TIMEOUT_WAIT) < NANO_MUTEX_FAIL)
	{
		return 0;
	}
#endif

	if (uart->config.use_interrupts && uart->recv_job && !int_state)
	{
		// job transfer is more efficient then byte transfer
		if (FAIL != task_id_get())
		{
			// the job might begin processing as soon as msg is set
			uart->job_rx.bytes_transferred = 0;
			uart->job_rx.bytes_requested = count;
			uart->job_rx.msg = msg;

			if (OK == uart->recv_job(uart))
			{
				count = uart->job_rx.bytes_transferred;

#ifdef NANO_RTOS
				mutex_unlock(&uart->mutex_rx);
#endif

				return count;
			}
		}
	}

	// byte transfer
	size_t c = 0;

	for (; (c < count) && !uart->status.stop; c++, msg++)
	{
		if (OK != uart->recv_byte(uart, msg))
		{
			break;
		}

		if (uart->config.idle_ends_recv)
		{
			c++;
			break;
		}
	}

#ifdef NANO_RTOS
	mutex_unlock(&uart->mutex_rx);
#endif

	return c;
}

int uart_wake_tx(struct uart_api * uart)
{
	if (uart->task_tx == FAIL)
	{
		return FAIL;
	}

	int b = task_wake(uart->task_tx);

	if (b != FAIL)
	{
		uart->task_tx = FAIL;
	}

	return b;
}

int uart_wake_rx(struct uart_api * uart)
{
	if (uart->task_rx == FAIL)
	{
		return FAIL;
	}

	int b = task_wake(uart->task_rx);

	if (b != FAIL)
	{
		uart->task_rx = FAIL;
	}

	return b;
}

// End of generic_uart.c *****************************************************
