/*
 * Copyright (c) 2020 EUROS Embedded Systems GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gpio_t.h"
#include <string.h>
#include <zephyr/kernel.h>


struct GPIO_t gpio;


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


int pin_get(struct PIN_obj_t pin)
{
	if (!pin.dev)
	{
		return 0;
	}

	return gpio_pin_get(pin.dev, pin.pin);
}

int pin_set(struct PIN_obj_t pin, int on)
{
	if (!pin.dev)
	{
		return FAIL;
	}

	return gpio_pin_set(pin.dev, pin.pin, on);
}

void flag_nrf91(void)
{
	pin_set(gpio.nrf9160_int, 1);
	k_usleep(10);
	pin_set(gpio.nrf9160_int, 0);
}
