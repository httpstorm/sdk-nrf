//****************************************************************************
//
// File:         gpio_t.h
// Author(s):    Georgi Valkov (GV)
// Project:      nRF52840 GPIO
// Description:  nRF52840 GPIO
//
// Copyright (C) 2022 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
//****************************************************************************

#ifndef _GPIO_H
#define _GPIO_H

#include <arch/cm4/nrf52840/nrf52840.h>


// PIN_CNF DIRECTION
typedef enum
{
	PIN_CNF_DIRECTION_INPUT = 0,
	PIN_CNF_DIRECTION_OUTPUT = 1,
} pin_dir_t;

// PIN_CNF INPUT
typedef enum
{
	PIN_CNF_INPUT_CONNECTED = 0,
	PIN_CNF_INPUT_DISCONNECTED = 1,
} pin_input_t;

// PIN_CNF PULL RESISTOR
typedef enum
{
	PIN_CNF_PULL_DISABLED = 0,
	PIN_CNF_PULL_DOWN = 1,
	PIN_CNF_PULL_UP = 3,
} pin_pull_t;

// PIN_CNF DRIVE STRENGTH CONTROL
typedef enum
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
typedef enum
{
	PIN_CNF_SENSE_DISABLED = 0,
	PIN_CNF_SENSE_HIGH = 2,
	PIN_CNF_SENSE_LOW = 3,
} pin_sense_t;

// PIN_PORT
typedef enum
{
	PIN_PORT_0 = 0,
	PIN_PORT_1 = 1,
} pin_port_t;

// PIN_LEVEL
typedef enum
{
	PIN_LEVEL_LOW = 0,
	PIN_LEVEL_HIGH = 1,
} pin_level_t;

// PIN_DETECT_MODE
typedef enum
{
	PIN_DETECT_MODE_DEFAULT = 0,
	PIN_DETECT_MODE_LDETECT = 1,
} pin_detect_mode_t;

// PIN_CNF
typedef struct
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
typedef struct
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
} pin_config_t;

// pin count
#define GPIO_PIN_COUNT(pin_config) (sizeof((pin_config)) / sizeof(*(pin_config)))


// init GPIO pins in list
void gpio_init_list(pin_config_t * list, size_t count);

// init GPIO pins
void gpio_init(void);


#endif /* _GPIO_H */
