// ***************************************************************************
//
// File:         gpio_t.c
// Author(s):    Georgi Valkov (GV)
// Project:      nRF52840 GPIO
// Description:  nordic nRF GPIO
//
// Copyright (C) 2022 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
// ***************************************************************************

// Include files *************************************************************

#include "gpio_t.h"
#include <string.h>
#include <zephyr/kernel.h>


// Macro definitions *********************************************************

// Types *********************************************************************

// Local Prototypes **********************************************************

// Global Prototypes *********************************************************

// Global variables **********************************************************

struct GPIO_t gpio;


// Functions *****************************************************************

// ***************************************************************************
// Function:      gpio_init
// Description:   configure GPIO pins
// Parameters:    -
// Return values: -
// Comments:      -
// ***************************************************************************
int gpio_init(void)
{
	int b = OK;
	size_t i = 0;

	memset(&gpio, 0, sizeof(gpio));

	if ((GPIO_COUNT_DEVS != GPIO_COUNT_PINS) || (GPIO_COUNT_DEVS != GPIO_COUNT_FLAGS))
	{
		printk("gpio_init()  gpio_devs, gpio_pins, and gpio_flags should have the same length\n");
		return FAIL;
	}

	for (i = 0; i < GPIO_COUNT_DEVS; i++)
	{
		gpio.list[i].dev = gpio_devs[0];
		gpio.list[i].pin = gpio_pins[0];

		// include GPIO_INPUT so we can read the state of the output using pin_get()
		if (
			!gpio.list[i].dev ||
			(gpio_pin_configure(
				gpio.list[i].dev,
				gpio.list[i].pin,
				gpio_flags[0] | GPIO_OUTPUT_INACTIVE | GPIO_INPUT
			) < 0)
			)
		{
			gpio.list[i].dev = NULL;
			b = FAIL;
		}
	}

	return b;
}


// ***************************************************************************
// Function:      pin_get
// Description:   get GPIO pin level
// Parameters:    pin           pin configuration
// Return values: pin level
// Comments:      -
// ***************************************************************************
int pin_get(const struct PIN_obj_t pin)
{
	if (!pin.dev)
	{
		return 0;
	}

	return gpio_pin_get(pin.dev, pin.pin);
}

// ***************************************************************************
// Function:      pin_set
// Description:   set GPIO pin
// Parameters:    pin           pin configuration
// Return values: -
// Comments:      -
// ***************************************************************************
int pin_set(const struct PIN_obj_t pin, int on)
{
	if (!pin.dev)
	{
		return FAIL;
	}

	return gpio_pin_set(pin.dev, pin.pin, on);
}

// ***************************************************************************
// Function:      flag_nrf91
// Description:   send a GPIO interrupt to nRF9160
// Parameters:    pin           pin configuration
// Return values: -
// Comments:      -
// ***************************************************************************
void flag_nrf91(void)
{
	pin_set(gpio.nrf9160_int, 1);
	k_usleep(10);
	pin_set(gpio.nrf9160_int, 0);
}

// End of gpio_t.c ***********************************************************
