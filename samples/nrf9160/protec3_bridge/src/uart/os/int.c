//****************************************************************************
//
// File:         int.c
// Author(s):    Georgi Valkov (GV)
// Project:      nano RTOS
// Description:  interrupt services
//
// Copyright (C) 2021 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
//****************************************************************************

// Include files *************************************************************

#include <int.h>


// Macro definitions *********************************************************

// Types *********************************************************************

// Local Prototypes **********************************************************

// Global variables **********************************************************

// use 16 when the system interrupt sources are defined from -15 to 0
// use 0 when the system interrupt sources are defined from 0 to 15
const size_t _ram_vectors_offset = (SysTick_IRQn < 0) ? 16 : 0;
const size_t _nvic_offset = 16 - _ram_vectors_offset;


// Functions *****************************************************************

#define NUM_SOURCES                  63
#define NVIC_CLRENA_BASE             0xE000E180
#define NVIC_SETENA_BASE             0xE000E100
#define NVIC_INTPRIORITY_BASE        0xE000E400


// ***************************************************************************
// Function:      int_source_disable
// Description:   disable an interrupt source
// Parameters:    int_number      interrupt number of the source
// Return values: OK              OK
//                FAIL            interrupt number not valid
// Comments:      -
// ***************************************************************************
int int_source_disable(uint32_t int_number)
{
	int_number -= _nvic_offset;

	if (int_number >= NUM_SOURCES)
	{
		// negative (system interrupt) or too large
		errno = EINVAL;
		return FAIL;
	}

	const uint32_t reg = int_number >> 5;
	const uint32_t mask = 0x1 << (int_number & 0x1f);
	volatile uint32_t * nvic_icer = (volatile uint32_t *)NVIC_CLRENA_BASE;
	nvic_icer[reg] = mask; // interrupt enable clear

	return OK;
}

// ***************************************************************************
// Function:      int_source_enable
// Description:   enable an interrupt source
// Parameters:    int_number      interrupt number of the source
// Return values: OK              OK
//                FAIL            interrupt number not valid
// Comments:      -
// ***************************************************************************
int int_source_enable(uint32_t int_number)
{
	int_number -= _nvic_offset;

	if (int_number >= NUM_SOURCES)
	{
		// negative (system interrupt) or too large
		errno = EINVAL;
		return FAIL;
	}

	const uint32_t reg = int_number >> 5;
	const uint32_t mask = 0x1 << (int_number & 0x1f);
	volatile uint32_t * nvic_iser = (volatile uint32_t *)NVIC_SETENA_BASE;
	nvic_iser[reg] = mask;

	return OK;
}

// ***************************************************************************
// Function:      int_source_set_priority, int_source_configure
// Description:   configure an interrupt source
// Parameters:    int_number      interrupt number of the source
//                priority        interrupt priority
// Return values: OK              OK
//                FAIL            interrupt number not valid
// Comments:      needs prior use of NVIC_PriorityGroupConfig
// ***************************************************************************
int int_source_configure(uint32_t int_number, uint8_t prio, uint8_t mode)
{
	int_number -= _nvic_offset;

	if (int_number >= NUM_SOURCES)
	{
		// negative (system interrupt) or too large
		errno = EINVAL;
		return FAIL;
	}

	*((uint8_t *)(NVIC_INTPRIORITY_BASE + int_number)) = (prio << 4) & 0xe0;

	return OK;
}

struct isr_t
{
	uint32_t param;
	uint32_t func;
};

// ***************************************************************************
// Function:      int_set_vector
// Description:   associate a function with an interrupt
// Parameters:    int_number      interrupt number
//                handler         new handler
// Return values: OK / FAIL
// Comments:      -
// ***************************************************************************
// if an argument is used, the handler function
// must be declared without __attribute__((interrupt))
int int_set_vector(uint32_t int_number, fn_int_handler int_handler)
{
	if (int_number >= NUM_SOURCES)
	{
		errno = EINVAL;
		return FAIL;
	}

	// func = nrfx_isr
	// param = actual handler e.g. nrfx_gpiote_irq_handler
	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_number];
	isr->param = 0;
	isr->func = (uint32_t)int_handler | 1;

	return OK;
}

// ***************************************************************************
// Function:      int_set_vector2
// Description:   associate a function with an interrupt
// Parameters:    int_number      interrupt number
//                handler         new handler
// Return values: OK / FAIL
// Comments:      -
// ***************************************************************************
int int_set_vector2(uint32_t int_number, fn_int_handler2 int_handler)
{
	if (int_number >= NUM_SOURCES)
	{
		errno = EINVAL;
		return FAIL;
	}

	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_number];
	isr->func = (uint32_t)int_handler | 1;

	return OK;
}

// ***************************************************************************
// Function:      int_get_vector
// Description:   return the current interrupt handler for the vector
// Parameters:    int_number      interrupt number
// Return values: fn_int_handler  function address
// Comments:      -
// ***************************************************************************
fn_int_handler int_get_vector(uint32_t int_number)
{
	if (int_number >= NUM_SOURCES)
	{
		errno = EINVAL;
		return NULL;
	}

	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_number];
	return (fn_int_handler)(isr->func & ~1);
}

// ***************************************************************************
// Function:      int_get_vector2
// Description:   return the current interrupt handler for the vector
// Parameters:    int_number      interrupt number
// Return values: fn_int_handler  function address
// Comments:      -
// ***************************************************************************
fn_int_handler2 int_get_vector2(uint32_t int_number)
{
	if (int_number >= NUM_SOURCES)
	{
		errno = EINVAL;
		return NULL;
	}

	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_number];
	return (fn_int_handler2)(isr->func & ~1);
}

// ***************************************************************************
// Function:      int_get_vector_arg
// Description:   return the current interrupt handler for the vector
// Parameters:    int_number      interrupt number
// Return values: fn_int_handler  function address
// Comments:      -
// ***************************************************************************
const void * int_get_vector_arg(uint32_t int_number)
{
	if (int_number >= NUM_SOURCES)
	{
		errno = EINVAL;
		return NULL;
	}

	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_number];
	return (const void *)isr->param;
}

// End of int.c **************************************************************
