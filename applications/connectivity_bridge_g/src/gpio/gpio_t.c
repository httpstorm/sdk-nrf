// ***************************************************************************
//
// File:         gpio_t.c
// Author(s):    Georgi Valkov (GV)
// Project:      nano RTOS
// Description:  nordic nRF GPIO
//
// Copyright (C) 2022 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
// ***************************************************************************

// Include files *************************************************************

#include <stddef.h>
#include <stdint.h>
#include "gpio_t.h"


// Macro definitions *********************************************************
#ifndef CONFIG_BOARD_NRF52840DK_NRF52840
#define CONFIG_BOARD_NRF52840DK_NRF52840 0
#endif

#ifndef CONFIG_BOARD_NRF9160DK_NRF52840
#define CONFIG_BOARD_NRF9160DK_NRF52840 0
#endif

#if !defined(CONFIG_BOARD_NRF9160DK_NRF9160_NS) && !defined(CONFIG_BOARD_NRF9160DK_NRF9160_S)
#define CONFIG_BOARD_NRF9160DK_NRF9160_S 0
#endif

#ifndef CONFIG_BOARD_THINGY91_NRF52840
#define CONFIG_BOARD_THINGY91_NRF52840 0
#endif

#ifndef CONFIG_BOARD_THINGY91_NRF9160
#define CONFIG_BOARD_THINGY91_NRF9160 0
#endif


// Types *********************************************************************

// Local Prototypes **********************************************************

// Global Prototypes *********************************************************

// Global variables **********************************************************

gpio_t pin_config[] =
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
#elif CONFIG_BOARD_NRF9160DK_NRF9160_NS || CONFIG_BOARD_NRF9160DK_NRF9160_S
#elif CONFIG_BOARD_THINGY91_NRF52840
#elif CONFIG_BOARD_THINGY91_NRF9160
	// configured automaticaly from DTS
#endif
};


// Functions *****************************************************************

// ***************************************************************************
// Function:      gpio_init
// Description:   configure the pin_config list of GPIO pins
// Parameters:    -
// Return values: -
// Comments:      -
// ***************************************************************************
void gpio_init(void)
{
	gpio_init_list(pin_config, GPIO_PIN_COUNT(pin_config));
}

// ***************************************************************************
// Function:      gpio_init_list
// Description:   configure a list of GPIO pins
// Parameters:    list          list of pin configurations
// Return values: -
// Comments:      -
// ***************************************************************************
void gpio_init_list(const gpio_t * list, size_t count)
{
	for (size_t i = 0; i < count; i++)
	{
		gpio_configure(&list[i]);
	}
}

// ***************************************************************************
// Function:      gpio_configure
// Description:   configure GPIO pin
// Parameters:    gpio          pin configuration
// Return values: -
// Comments:      -
// ***************************************************************************
void gpio_configure(const gpio_t * gpio)
{
	NRF_GPIO_Type * port = GPIO_PORT(gpio->port);
	const uint32_t pin = gpio->pin;
	const uint32_t mask = 1 << pin;

	if (gpio->level == PIN_LEVEL_HIGH)
	{
		port->OUTSET = mask;
	}
	else
	{
		port->OUTCLR = mask;
	}

	port->PIN_CNF[pin] = gpio->pin_cnf_raw;

	if (gpio->pin_cnf.sense)
	{
		// wait for the latch to set and then clear it
		size_t j = 4;

		while (j--)
			;

		port->LATCH = mask;
	}
}

// ***************************************************************************
// Function:      gpio_set_high
// Description:   set GPIO pin high
// Parameters:    gpio          pin configuration
// Return values: -
// Comments:      -
// ***************************************************************************
void gpio_set_high(const gpio_t * gpio)
{
	NRF_GPIO_Type * port = GPIO_PORT(gpio->port);
	const uint32_t pin = gpio->pin;
	const uint32_t mask = 1 << pin;
	port->OUTSET = mask;
}

// ***************************************************************************
// Function:      gpio_set_low
// Description:   set GPIO pin low
// Parameters:    gpio          pin configuration
// Return values: -
// Comments:      -
// ***************************************************************************
void gpio_set_low(const gpio_t * gpio)
{
	NRF_GPIO_Type * port = GPIO_PORT(gpio->port);
	const uint32_t pin = gpio->pin;
	const uint32_t mask = 1 << pin;
	port->OUTCLR = mask;
}

// ***************************************************************************
// Function:      gpio_get
// Description:   get GPIO pin level
// Parameters:    gpio          pin configuration
// Return values: pin level
// Comments:      -
// ***************************************************************************
uint32_t gpio_get(const gpio_t * gpio)
{
	NRF_GPIO_Type * port = GPIO_PORT(gpio->port);
	const uint32_t pin = gpio->pin;
	const uint32_t mask = 1 << pin;
	return (port->IN & mask) ? 1 : 0;
}

// End of gpio_t.c ***********************************************************
