//****************************************************************************
//
// File:         int.h
// Author(s):    Georgi Valkov (GV)
// Project:      nano RTOS
// Description:  interrupt services
//
// Copyright (C) 2021 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
//****************************************************************************

#ifndef _INT_H
#define _INT_H

// Include files *************************************************************

#include <os_layer.h>


// Macros ********************************************************************

// Types *********************************************************************

_BEGIN_CPLUSPLUS

typedef uint32_t	int_state_t;

typedef void (* fn_int_handler)(void);
typedef void (* fn_int_handler2)(const void *);


// Global Prototypes *********************************************************

// Function prototypes *******************************************************

// low level interrupt services
void int_disable(void);
void int_enable(void);

int_state_t	int_disable_nested(void);
void int_enable_nested(int_state_t int_state_old);

int_state_t int_get_faultmask_or_primask(void);
int_state_t int_get_faultmask(void);
int_state_t int_get_primask(void);
#define int_is_disabled int_get_primask
int_state_t int_unavailable(void);
int_state_t int_get_active(void);

// interrupt services
int int_source_disable(uint16_t int_num);
int int_source_enable(uint16_t int_num);
int int_source_configure(uint16_t int_num, uint16_t prio, uint16_t mode);

int int_set_vector(uint16_t int_num, fn_int_handler int_handler);
int int_set_vector2(uint16_t int_num, fn_int_handler2 int_handler);
fn_int_handler int_get_vector(uint16_t int_num);
fn_int_handler2 int_get_vector2(uint16_t int_num);
const void * int_get_vector_arg(uint16_t int_num);

_END_CPLUSPLUS
#endif // _INT_H
