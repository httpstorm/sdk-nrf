//****************************************************************************
//
// File:         os_layer.h
// Author(s):    Georgi Valkov (GV)
// Project:      nano RTOS
// Description:  OS compatibility layer
//
// Copyright (C) 2022 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
//****************************************************************************

#ifndef OS_LAYER_H
#define OS_LAYER_H

// Include files *************************************************************

#if defined (EUROS) || defined (OSEK)
#include <os_euros.h>
#elif defined (__ZEPHYR__)
#include "os_zephyr.h"
#elif defined (NANO_RTOS)
#include <os_nano.h>
#else
#error unsupported OS
#endif

#endif // OS_LAYER_H
