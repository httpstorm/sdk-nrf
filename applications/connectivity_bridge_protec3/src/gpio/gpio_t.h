#ifndef GUARD_GPIO_T_H
#define GUARD_GPIO_T_H

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>


#define OK 0
#define FAIL -1


#define NRF9160_INT_NODE DT_ALIAS(nrf9160_int)

#if DT_NODE_HAS_STATUS(NRF9160_INT_NODE, okay)
#define NRF9160_INT_NAME  DT_GPIO_LABEL(NRF9160_INT_NODE, gpios)
#define NRF9160_INT_PIN   DT_GPIO_PIN(NRF9160_INT_NODE, gpios)
#define NRF9160_INT_FLAGS DT_GPIO_FLAGS(NRF9160_INT_NODE, gpios)

static const char * gpio_names[] = { NRF9160_INT_NAME };
static const gpio_pin_t gpio_pins[] = { NRF9160_INT_PIN };
static const gpio_flags_t gpio_flags[] = { NRF9160_INT_FLAGS };

#define GPIO_COUNT_NAMES (sizeof(gpio_names) / sizeof(*gpio_names))
#define GPIO_COUNT_PINS  (sizeof(gpio_pins)  / sizeof(*gpio_pins))
#define GPIO_COUNT_FLAGS (sizeof(gpio_flags) / sizeof(*gpio_flags))
#else
#error "Unsupported board: nrf9160_int devicetree alias is not defined"
#endif


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

		struct PIN_obj_t list[GPIO_COUNT_NAMES];
	};
};

extern struct GPIO_t gpio;


int gpio_init(void);
int pin_get(struct PIN_obj_t pin);
int pin_set(struct PIN_obj_t pin, int on);
void flag_nrf91(void);


#endif /* GUARD_GPIO_T_H */
