//****************************************************************************
//
// File:         int.s
// Author(s):    Georgi Valkov (GV)
// Project:      nano RTOS
// Description:  interrupt services
//
// Copyright (C) 2022 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
//****************************************************************************

// Include files *************************************************************

// Macro definitions *********************************************************

// exported function in separate section
	.macro _global_func func_name
	.text
	.code 16
	.balign 2
	.thumb_func
	.global \func_name
	.type \func_name,#function
\func_name:
	.endm

// Types *********************************************************************

// Global variables **********************************************************

// Functions *****************************************************************

	.text
	.thumb
	.balign 2
	.syntax unified

//# disable interrupts
//# void int_disable(void)
_global_func int_disable
	cpsid i
	bx lr

//# enable interrupts
//# void int_enable(void)
_global_func int_enable
	cpsie i
	bx lr

//# disable interrupts, calls can be nested
//# return the previous state
//# int_state_t int_disable_nested(void)
_global_func int_disable_nested
	mrs   r0, primask                   //# r0 = interrupts disabled flag
	cpsid i
	bx lr

//# restore interrupts to int_state, calls can be nested
//# void int_enable_nested(int_state_t int_state)
_global_func int_enable_nested
	msr   primask, r0                   //# interrupts disabled flag = r0
	bx lr


//# returns (faultmask | primask) faults or interrupts disabled
//# tIntState int_get_faultmask_or_primask(void)
_global_func int_get_faultmask_or_primask
	mrs   r0, faultmask                 //# r0 = faults disabled flag
	mrs   r1, primask                   //# r1 = interrupts disabled flag
	orr   r0, r1                        //# r0 |= r1
	bx lr

//# returns faultmask which is 1 when faults are disabled
//# tIntState int_get_faultmask(void)
_global_func int_get_faultmask
	mrs   r0, faultmask                 //# r0 = faults disabled flag
	bx lr

//# returns primask which is 1 when interrupts are disabled
//# tIntState int_get_primask(void)
_global_func int_get_primask
	mrs   r0, primask                   //# r0 = interrupts disabled flag
	bx lr

//# returns (faultmask | primask | currently active interrupt number)
//# faults or interrupts disabled or running as an interrupt handler
//# int_state_t int_unavailable(void)
_global_func int_unavailable
	mrs   r0, faultmask                 //# r0 = faults disabled flag
	mrs   r1, primask                   //# r1 = interrupts disabled flag
	mrs   r2, ipsr                      //# r2 = ipsr  currently active ISR
	orr   r0, r1                        //# r0 |= r1
	orr   r0, r2                        //# r0 |= r2
	bx lr

//# returns the currently active interrupt number
//# int_state_t int_get_active(void)
_global_func int_get_active
	mrs   r0, ipsr                      //# r0 = ipsr  currently active ISR
	bx lr

// End of int.s **************************************************************
.end
