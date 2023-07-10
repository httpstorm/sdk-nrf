// ***************************************************************************
//
// File:         os_zephyr.h
// Author(s):    Georgi Valkov (GV)
// Project:      compatibility layer for Zephyr
// Description:  OS compatibility layer
//
// Copyright (C) 2021-2023 by httpstorm.com
//                      Tselina str. 6, 4000 Plovdiv, Bulgaria
//                      e-mail: gvalkov@gmail.com
//
// ***************************************************************************

#ifndef OS_ZEPHYR_H
#define OS_ZEPHYR_H

// Include files *************************************************************

#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>


// Macros ********************************************************************

#define FAIL           -1
#define OK              0

#ifdef __cplusplus
#  define _BEGIN_CPLUSPLUS   extern "C" {
#  define _END_CPLUSPLUS                }
#else
#  define _BEGIN_CPLUSPLUS
#  define _END_CPLUSPLUS
#endif

#include "int.h"


// Types *********************************************************************

// Local Prototypes **********************************************************

// Global Prototypes *********************************************************

// Function prototypes *******************************************************

int task_id_get(void);
int task_wait(void);
int task_wake(int task_id);


// Global variables **********************************************************

// End of file ***************************************************************

#endif // OS_ZEPHYR_H
