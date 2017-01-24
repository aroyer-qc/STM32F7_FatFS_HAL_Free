/* Includes ---------------------------------------------------------------------------------------------------------*/

#include "stm32f746xx.h"
#include "io.h"

/* ------------------------------------------------------------------------------------------------------------------*/

/** -----------------------------------------------------------------------------------------------------------------*/
/**		Initialize the IO pin according to parameter
  *
  * @brief  Basic pin initialization for this demo
  *
  * @note   Basic initialization for IO pin to remove the HAL (HORROR AT LARGE) because I am HALlergic to HAL
  *         Analog not tested, and also may not support all configuration and may need modification
  *         This was only bring-up for this demo
  *
  */
void IO_PinInit(GPIO_TypeDef* pPort, uint32_t ClockEnable, uint32_t PinNumber, uint32_t PinMode, uint32_t PinType, uint32_t PinSpeed, uint32_t MultiState)
{
    // GPIO clock enable
    RCC->AHB1ENR |= ClockEnable;

    pPort->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_PIN_MASK << (PinNumber << 1));
    pPort->OSPEEDR |=  (uint32_t)(PinSpeed << (PinNumber << 1));

    if((PinMode == GPIO_MODER_OUTPUT) || (PinMode == GPIO_MODER_ALTERNATE))
    {
        if(PinMode == GPIO_MODER_OUTPUT)
        {
            // Preset initial state
            if(MultiState == 0)   IO_SetPinLow(pPort, PinNumber);
            else                  IO_SetPinHigh(pPort, PinNumber);
        }

        pPort->OTYPER  &= ~(uint32_t)(GPIO_TYPE_PIN_DRIVE_MASK << PinNumber);        // Reset bit for Push-Pull
        pPort->OTYPER  |=  (uint32_t)(PinType << PinNumber);                         // Set new type
    }

    if(PinMode == GPIO_MODER_INPUT)
    {
        pPort->PUPDR  &= ~(uint32_t)(GPIO_TYPE_PIN_PULL_MASK << PinNumber);          // Reset bit for Pull Up
        pPort->PUPDR  |=  (uint32_t)(PinType << PinNumber);                          // Set new type
    }
    else if(PinMode == GPIO_MODER_ALTERNATE)
    {
        if(PinNumber < 8)
        {
            pPort->AFR[0] &= ~(uint32_t)(GPIO_AF_MASK << (PinNumber << 2));
            pPort->AFR[0] |=  (uint32_t)(MultiState   << (PinNumber << 2));
        }
        else
        {
            pPort->AFR[1] &= ~(uint32_t)(GPIO_AF_MASK << ((PinNumber - 8) << 2));
            pPort->AFR[1] |=  (uint32_t)(MultiState   << ((PinNumber - 8) << 2));
        }
    }

    pPort->MODER   &= ~(uint32_t)(GPIO_MODER_PIN_MASK << (PinNumber << 1));
    pPort->MODER   |=  (uint32_t)(PinMode << (PinNumber << 1));
}

/* ------------------------------------------------------------------------------------------------------------------*/
