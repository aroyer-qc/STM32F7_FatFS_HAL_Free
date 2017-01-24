#ifndef __BSP_H__
#define __BSP_H__

/* Includes ---------------------------------------------------------------------------------------------------------*/

#include <stdint.h>
#include "stm32f7xx.h"
#include "io.h"

/* Defines ----------------------------------------------------------------------------------------------------------*/

#define LED_PORT		    GPIOI
#define LED1				GPIO_PIN_1

#define ledOn()             IO_SetPinHigh(LED_PORT, LED1)
#define ledOff()        	IO_SetPinLow(LED_PORT, LED1)
#define ledToggle()         IO_TogglePinValue(LED_PORT, LED1)

// Use this macro to remove warning on unused variables
#define VAR_UNUSED(v)       ((void)(v))

/* ------------------------------------------------------------------------------------------------------------------*/

extern uint32_t Tick;

/* ------------------------------------------------------------------------------------------------------------------*/

void    Initialize  (void);

/* ------------------------------------------------------------------------------------------------------------------*/

#endif // __BSP_H__
