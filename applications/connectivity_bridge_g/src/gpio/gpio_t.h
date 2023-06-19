// ***************************************************************************
//
// File:         gpio_t.h
// Author(s):    Georgi Valkov (GV)
// Project:      nano RTOS
// Description:  nRF52840 GPIO
//
// Copyright (C) 2022 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
// ***************************************************************************

#ifndef GPIO_T_H
#define GPIO_T_H


// Include files *************************************************************

#if CONFIG_BOARD_NRF9160DK_NRF9160_NS || CONFIG_BOARD_NRF9160DK_NRF9160_S || CONFIG_BOARD_THINGY91_NRF9160
#include <arch/nordic/nrf9160.h>
#else
#include <arch/nordic/nrf52840.h>
#endif


// Macros ********************************************************************

// pin count
#define GPIO_PIN_COUNT(pin_config) (sizeof((pin_config)) / sizeof(*(pin_config)))

#if CONFIG_BOARD_NRF9160DK_NRF9160_S || CONFIG_BOARD_THINGY91_NRF9160
#define GPIO_PORT(port) NRF_P0_S
#elif CONFIG_BOARD_NRF9160DK_NRF9160_NS
#define GPIO_PORT(port) NRF_P0_NS
#else
#define GPIO_PORT(port) ((port == PIN_PORT_1) ? NRF_P1 : NRF_P0)
#endif


// Types *********************************************************************

// PIN_CNF DIRECTION
typedef enum pin_dir_t
{
	PIN_CNF_DIRECTION_INPUT = 0,
	PIN_CNF_DIRECTION_OUTPUT = 1,
} pin_dir_t;

// PIN_CNF INPUT
typedef enum pin_input_t
{
	PIN_CNF_INPUT_CONNECTED = 0,
	PIN_CNF_INPUT_DISCONNECTED = 1,
} pin_input_t;

// PIN_CNF PULL RESISTOR
typedef enum pin_pull_t
{
	PIN_CNF_PULL_DISABLED = 0,
	PIN_CNF_PULL_DOWN = 1,
	PIN_CNF_PULL_UP = 3,
} pin_pull_t;

// PIN_CNF DRIVE STRENGTH CONTROL
typedef enum pin_drive_t
{
	PIN_CNF_DRIVE_STANDARD_0_STANDARD_1 = 0,
	PIN_CNF_DRIVE_HIGH_0_STANDARD_1 = 1,
	PIN_CNF_DRIVE_STANDARD_0_HIGH_1 = 2,
	PIN_CNF_DRIVE_HIGH_0_HIGH_1 = 3,
	PIN_CNF_DRIVE_DISCONNECT_0_STANDARD_1 = 4,
	PIN_CNF_DRIVE_DISCONNECT_0_HIGH_1 = 5,
	PIN_CNF_DRIVE_STANDARD_0_DISCONNECT_1 = 6,
	PIN_CNF_DRIVE_HIGH_0_DISCONNECT_1 = 7,
} pin_drive_t;

// PIN_CNF SENSE MODE
typedef enum pin_sense_t
{
	PIN_CNF_SENSE_DISABLED = 0,
	PIN_CNF_SENSE_HIGH = 2,
	PIN_CNF_SENSE_LOW = 3,
} pin_sense_t;

// PIN_PORT
typedef enum pin_port_t
{
	PIN_PORT_0 = 0,
	PIN_PORT_1 = 1,
} pin_port_t;

// PIN_LEVEL
typedef enum pin_level_t
{
	PIN_LEVEL_LOW = 0,
	PIN_LEVEL_HIGH = 1,
} pin_level_t;

// PIN_DETECT_MODE
typedef enum pin_detect_mode_t
{
	PIN_DETECT_MODE_DEFAULT = 0,
	PIN_DETECT_MODE_LDETECT = 1,
} pin_detect_mode_t;

// PIN_CNF
typedef struct pin_cnf_t
{
	pin_dir_t dir : 1;      // 00-00 dir
	pin_input_t input : 1;  // 01-01 input
	pin_pull_t pull : 2;    // 02-03 pull
	uint32_t : 4;           // 04-07
	pin_drive_t drive : 3;  // 08-0a drive
	uint32_t : 5;           // 0b-0f
	pin_sense_t sense : 2;  // 10-11 sense
	uint32_t : 14;          // 12-1f
} pin_cnf_t;

// PIN_CONFIG
typedef struct gpio_t
{
	pin_port_t port : 1;
	uint32_t : 7;
	uint32_t pin : 5;
	uint32_t : 3;
	pin_level_t level : 1;
	uint32_t : 15;

	union
	{
		pin_cnf_t pin_cnf;
		uint32_t pin_cnf_raw;
	};
} gpio_t;


// Local Prototypes **********************************************************

// Global Prototypes *********************************************************

// Global variables **********************************************************

// Function prototypes *******************************************************

// init GPIO pins
void gpio_init(void);

// init GPIO pins in list
void gpio_init_list(const gpio_t * list, size_t count);

void gpio_configure(const gpio_t * gpio);
void gpio_set_high(const gpio_t * gpio);
void gpio_set_low(const gpio_t * gpio);
uint32_t gpio_get(const gpio_t * gpio);

#endif

// End of gpio_t.h ***********************************************************
