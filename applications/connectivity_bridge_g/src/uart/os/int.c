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

// Functions *****************************************************************

#define NUM_SOURCES                  63
#define NVIC_CLRENA_BASE             0xE000E180
#define NVIC_SETENA_BASE             0xE000E100
#define NVIC_INTPRIORITY_BASE        0xE000E400


int int_source_disable(uint16_t int_num)
{
	if (int_num <= 15)
	{
		// 0-15 system vectors
		return OK;
	}

	if (int_num >= NUM_SOURCES)
	{
		errno = EINVAL;
		return FAIL;
	}

	int_num -= 16;

	int index = (uint32_t)int_num >> 5;
	volatile uint32_t * nvic_icer = (volatile uint32_t *)NVIC_CLRENA_BASE;
	nvic_icer[index] = (uint32_t)(1 << ((uint32_t)int_num & 0x1f));

	return OK;
}

int int_source_enable(uint16_t int_num)
{
	if (int_num <= 15)
	{
		// 0-15 system vectors
		return OK;
	}

	if (int_num >= NUM_SOURCES)
	{
		errno = EINVAL;
		return FAIL;
	}

	int_num -= 16;

	int index = (((uint32_t)int_num) >> 5);
	volatile uint32_t * nvic_iser = (volatile uint32_t *)NVIC_SETENA_BASE;
	nvic_iser[index] = (uint32_t)(1 << (((uint32_t)int_num) & 0x1f));

	return OK;
}

int int_source_configure(uint16_t int_num, uint16_t prio, uint16_t mode)
{
	if (int_num <= 15)
	{
		// 0-15 system vectors
		return OK;
	}

	if (int_num >= NUM_SOURCES)
	{
		errno = EINVAL;
		return FAIL;
	}

	int_num -= 16;

	*((uint8_t *)(NVIC_INTPRIORITY_BASE + int_num)) = (prio << 4) & 0xe0;

	return OK;
}

struct isr_t
{
	uint32_t param;
	uint32_t func;
};

// if an argument is used, the handler function
// must be declared without __attribute__((interrupt))
int int_set_vector(uint16_t int_num, fn_int_handler int_handler)
{
	if (int_num >= NUM_SOURCES)
	{
		errno = EINVAL;
		return FAIL;
	}

	int_num -= 16;

	// func = nrfx_isr
	// param = actual handler e.g. nrfx_gpiote_irq_handler
	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_num];
	isr->param = 0;
	isr->func = (uint32_t)int_handler | 1;

	return OK;
}

int int_set_vector2(uint16_t int_num, fn_int_handler2 int_handler)
{
	if (int_num >= NUM_SOURCES)
	{
		errno = EINVAL;
		return FAIL;
	}

	int_num -= 16;

	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_num];
	isr->func = (uint32_t)int_handler | 1;

	return OK;
}

fn_int_handler int_get_vector(uint16_t int_num)
{
	if (int_num >= NUM_SOURCES)
	{
		errno = EINVAL;
		return NULL;
	}

	int_num -= 16;

	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_num];
	return (fn_int_handler)(isr->func & ~1);
}

fn_int_handler2 int_get_vector2(uint16_t int_num)
{
	if (int_num >= NUM_SOURCES)
	{
		errno = EINVAL;
		return NULL;
	}

	int_num -= 16;

	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_num];
	return (fn_int_handler2)(isr->func & ~1);
}

const void * int_get_vector_arg(uint16_t int_num)
{
	if (int_num >= NUM_SOURCES)
	{
		errno = EINVAL;
		return NULL;
	}

	int_num -= 16;

	struct isr_t * isr = (struct isr_t *)&_sw_isr_table[int_num];
	return (const void *)isr->param;
}

// End of int.c **************************************************************
