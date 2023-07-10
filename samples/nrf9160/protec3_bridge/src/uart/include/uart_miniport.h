// ***************************************************************************
//
// File:         uart_miniport.h
// Author(s):    Georgi Valkov (GV)
// Project:      UART miniport driver for nrf52840
// Description:  hadrware specific definitions and types
//
// Copyright (C) 2020 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
// ***************************************************************************

#ifndef UART_MINIPORT_H
#define UART_MINIPORT_H


#if !CONFIG_NRFX_UARTE
#define USE_UART_MINIPORT 1
#endif

#if USE_UART_MINIPORT
#define UART_INTERFACE 0
#include <sys/types.h>
#include <miniport/uart/nrf52840_uart.h>
#include <miniport/uart/generic_uart.h>
#include "../../gpio/gpio_t.h"
#endif


#endif /* UART_MINIPORT_H */
