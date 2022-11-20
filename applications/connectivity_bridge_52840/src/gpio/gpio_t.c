//****************************************************************************
//
// File:         gpio_t.c
// Author(s):    Georgi Valkov (GV)
// Project:      nRF52840 GPIO
// Description:  nRF52840 GPIO
//
// Copyright (C) 2022 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
//****************************************************************************

// Include files *************************************************************
#include <stddef.h>
#include <stdint.h>
#include "gpio_t.h"


// Macro definitions *********************************************************
#ifndef CONFIG_BOARD_NRF52840DK_NRF52840
#define CONFIG_BOARD_NRF52840DK_NRF52840 1
#endif

#ifndef CONFIG_BOARD_NRF9160DK_NRF52840
#define CONFIG_BOARD_NRF9160DK_NRF52840 1
#endif


// Types *********************************************************************

pin_config_t pin_config[] =
{
#if CONFIG_BOARD_NRF52840DK_NRF52840
	// P0.13             LED0
	{
		.port = 0,
		.pin = 13,
		.level = PIN_LEVEL_HIGH,
		.pin_cnf =
		{
			.dir = PIN_CNF_DIRECTION_OUTPUT,
			.input = PIN_CNF_INPUT_DISCONNECTED,
			.pull = PIN_CNF_PULL_DISABLED,
			.drive = PIN_CNF_DRIVE_STANDARD_0_STANDARD_1,
			.sense = PIN_CNF_SENSE_DISABLED
		}
	},
	// P0.14             LED1
	{
		.port = 0,
		.pin = 14,
		.level = PIN_LEVEL_HIGH,
		.pin_cnf =
		{
			.dir = PIN_CNF_DIRECTION_OUTPUT,
			.input = PIN_CNF_INPUT_DISCONNECTED,
			.pull = PIN_CNF_PULL_DISABLED,
			.drive = PIN_CNF_DRIVE_STANDARD_0_STANDARD_1,
			.sense = PIN_CNF_SENSE_DISABLED
		}
	},
	// P0.15             LED2
	{
		.port = 0,
		.pin = 15,
		.level = PIN_LEVEL_HIGH,
		.pin_cnf =
		{
			.dir = PIN_CNF_DIRECTION_OUTPUT,
			.input = PIN_CNF_INPUT_DISCONNECTED,
			.pull = PIN_CNF_PULL_DISABLED,
			.drive = PIN_CNF_DRIVE_STANDARD_0_STANDARD_1,
			.sense = PIN_CNF_SENSE_DISABLED
		}
	},
	// P0.16             LED3
	{
		.port = 0,
		.pin = 16,
		.level = PIN_LEVEL_HIGH,
		.pin_cnf =
		{
			.dir = PIN_CNF_DIRECTION_OUTPUT,
			.input = PIN_CNF_INPUT_DISCONNECTED,
			.pull = PIN_CNF_PULL_DISABLED,
			.drive = PIN_CNF_DRIVE_STANDARD_0_STANDARD_1,
			.sense = PIN_CNF_SENSE_DISABLED
		}
	},
	// P0.11          BUTTON0
	{
		.port = 0,
		.pin = 11,
		.level = PIN_LEVEL_HIGH,
		.pin_cnf =
		{
			.dir = PIN_CNF_DIRECTION_INPUT,
			.input = PIN_CNF_INPUT_CONNECTED,
			.pull = PIN_CNF_PULL_UP,
			.drive = PIN_CNF_DRIVE_STANDARD_0_STANDARD_1,
			.sense = PIN_CNF_SENSE_LOW
		}
	},
	// P0.12          BUTTON1
	{
		.port = 0,
		.pin = 12,
		.level = PIN_LEVEL_HIGH,
		.pin_cnf =
		{
			.dir = PIN_CNF_DIRECTION_INPUT,
			.input = PIN_CNF_INPUT_CONNECTED,
			.pull = PIN_CNF_PULL_UP,
			.drive = PIN_CNF_DRIVE_STANDARD_0_STANDARD_1,
			.sense = PIN_CNF_SENSE_LOW
		}
	},
	// P0.24          BUTTON2
	{
		.port = 0,
		.pin = 24,
		.level = PIN_LEVEL_HIGH,
		.pin_cnf =
		{
			.dir = PIN_CNF_DIRECTION_INPUT,
			.input = PIN_CNF_INPUT_CONNECTED,
			.pull = PIN_CNF_PULL_UP,
			.drive = PIN_CNF_DRIVE_STANDARD_0_STANDARD_1,
			.sense = PIN_CNF_SENSE_LOW
		}
	},
	// P0.25          BUTTON3
	{
		.port = 0,
		.pin = 25,
		.level = PIN_LEVEL_HIGH,
		.pin_cnf =
		{
			.dir = PIN_CNF_DIRECTION_INPUT,
			.input = PIN_CNF_INPUT_CONNECTED,
			.pull = PIN_CNF_PULL_UP,
			.drive = PIN_CNF_DRIVE_STANDARD_0_STANDARD_1,
			.sense = PIN_CNF_SENSE_LOW
		}
	},
#elif CONFIG_BOARD_NRF9160DK_NRF52840
#endif
};


// Local Prototypes **********************************************************

// Global variables **********************************************************

// Functions *****************************************************************

void gpio_init_list(pin_config_t * list, size_t count)
{
	for (size_t i = 0; i < count; i++)
	{
		const pin_config_t * item = &list[i];
		NRF_GPIO_Type * port = (item->port == PIN_PORT_1) ? NRF_P1 : NRF_P0;
		const uint32_t pin = item->pin;
		const uint32_t mask = 1 << pin;

		if (item->level == PIN_LEVEL_HIGH)
		{
			port->OUTSET = mask;
		}
		else
		{
			port->OUTCLR = mask;
		}

		port->PIN_CNF[pin] = item->pin_cnf_raw;

		if (item->pin_cnf.sense)
		{
			// wait for the latch to set and then clear it
			size_t j = 4;

			while (j--)
				;

			port->LATCH = mask;
		}
	}
}

void gpio_init(void)
{
	gpio_init_list(pin_config, GPIO_PIN_COUNT(pin_config));
}


// End of hw_init.c **********************************************************
