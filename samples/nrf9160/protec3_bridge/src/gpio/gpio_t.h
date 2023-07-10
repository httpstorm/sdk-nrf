// ***************************************************************************
//
// File:         gpio_t.h
// Author(s):    Georgi Valkov (GV)
// Project:      nRF52840 GPIO
// Description:  nordic nRF GPIO
//
// Copyright (C) 2022 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
// ***************************************************************************

#ifndef GPIO_T_H
#define GPIO_T_H

// Include files *************************************************************

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>


// Macros ********************************************************************

#define OK 0
#define FAIL -1


#define NRF9160_INT_NODE DT_ALIAS(nrf9160_int)

#if DT_NODE_HAS_STATUS(NRF9160_INT_NODE, okay)
#define NRF9160_INT_DEV   DEVICE_DT_GET(DT_GPIO_CTLR(NRF9160_INT_NODE, gpios))
#define NRF9160_INT_PIN   DT_GPIO_PIN(NRF9160_INT_NODE, gpios)
#define NRF9160_INT_FLAGS DT_GPIO_FLAGS(NRF9160_INT_NODE, gpios)

static const struct device * gpio_devs[] = { NRF9160_INT_DEV };
static const gpio_pin_t gpio_pins[] = { NRF9160_INT_PIN };
static const gpio_flags_t gpio_flags[] = { NRF9160_INT_FLAGS };

#define GPIO_COUNT_DEVS  (sizeof(gpio_devs)  / sizeof(*gpio_devs))
#define GPIO_COUNT_PINS  (sizeof(gpio_pins)  / sizeof(*gpio_pins))
#define GPIO_COUNT_FLAGS (sizeof(gpio_flags) / sizeof(*gpio_flags))
#else
#error "Unsupported board: nrf9160_int devicetree alias is not defined"
#endif


// Types *********************************************************************

struct PIN_obj_t
{
	const struct device * dev;
	gpio_pin_t pin;
};

struct GPIO_t
{
	union
	{
		struct
		{
			struct PIN_obj_t nrf9160_int;
		};

		struct PIN_obj_t list[GPIO_COUNT_DEVS];
	};
};


// Local Prototypes **********************************************************

// Global Prototypes *********************************************************

// Global variables **********************************************************

extern struct GPIO_t gpio;


// Function prototypes *******************************************************

// init GPIO pins
int gpio_init(void);
int pin_get(const struct PIN_obj_t pin);
int pin_set(const struct PIN_obj_t pin, int on);
void flag_nrf91(void);

#endif

// End of gpio_t.h ***********************************************************
