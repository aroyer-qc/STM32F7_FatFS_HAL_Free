#ifndef _STM32F7_IO_H__
#define _STM32F7_IO_H__

/* ------------------------------------------------------------------------------------------------------------------*/

#define GPIO_PIN_0		                0
#define GPIO_PIN_1		                1
#define GPIO_PIN_2		                2
#define GPIO_PIN_3		                3
#define GPIO_PIN_4		                4
#define GPIO_PIN_5		                5
#define GPIO_PIN_6		                6
#define GPIO_PIN_7		                7
#define GPIO_PIN_8		                8
#define GPIO_PIN_9		                9
#define GPIO_PIN_10		                10
#define GPIO_PIN_11		                11
#define GPIO_PIN_12		                12
#define GPIO_PIN_13		                13
#define GPIO_PIN_14		                14
#define GPIO_PIN_15		                15

#define GPIO_OSPEEDR_PIN_MASK           ((uint32_t)0x00000003)
#define GPIO_OSPEEDR_LOW                ((uint32_t)0x00000000)
#define GPIO_OSPEEDR_MEDIUM             ((uint32_t)0x00000001)
#define GPIO_OSPEEDR_HIGH               ((uint32_t)0x00000002)
#define GPIO_OSPEEDR_VERY_HIGH          ((uint32_t)0x00000003)

#define GPIO_MODER_PIN_MASK             ((uint32_t)0x00000003)
#define GPIO_MODER_INPUT                ((uint32_t)0x00000000)
#define GPIO_MODER_OUTPUT               ((uint32_t)0x00000001)
#define GPIO_MODER_ALTERNATE            ((uint32_t)0x00000002)
#define GPIO_MODER_ANALOG               ((uint32_t)0x00000003)

#define GPIO_TYPE_PIN_DRIVE_MASK        ((uint32_t)0x00000001)
#define GPIO_TYPE_PIN_PP                ((uint32_t)0x00000000)
#define GPIO_TYPE_PIN_OD                ((uint32_t)0x00000001)
#define GPIO_TYPE_PIN_PULL_MASK         ((uint32_t)0x00000003)
#define GPIO_TYPE_PIN_NO_PULL           ((uint32_t)0x00000000)
#define GPIO_TYPE_PIN_PULL_UP           ((uint32_t)0x00000001)
#define GPIO_TYPE_PIN_PULL_DOWN         ((uint32_t)0x00000002)

#define GPIO_AF_MASK                    ((uint32_t)0x0000000F)

/* ------------------------------------------------------------------------------------------------------------------*/

/**
 * @brief  Sets pin(s) low
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin low
 * @param  GPIO_Pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them low
 */
#define IO_SetPinLow(GPIOx, GPIO_Pin)			((GPIOx)->BSRR = (uint32_t)( (1 << GPIO_Pin) << 16 ))

/**
 * @brief  Sets pin(s) high
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin high
 * @param  GPIO_Pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them high
 */
#define IO_SetPinHigh(GPIOx, GPIO_Pin)			((GPIOx)->BSRR = (uint32_t)( (1 << GPIO_Pin) ))

/**
 * @brief  Sets value to entire GPIO PORT
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set value
 * @param  value: Value for GPIO OUTPUT data
 */
#define IO_SetPortValue(GPIOx, value)			((GPIOx)->ODR = (value))

/**
 * @brief  Toggles pin(s)
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to toggle pin value
 * @param  GPIO_Pin: Select GPIO pin(s). You can select more pins with | (OR) operator to toggle them all at a time
 */
#define IO_TogglePinValue(GPIOx, GPIO_Pin)		((GPIOx)->ODR ^= ( (1 << GPIO_Pin) ))


/**
 * @brief  Sets pin(s) value
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin value
 * @param  GPIO_Pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them value
 * @param  val: If parameter is 0 then pin will be low, otherwise high
 */
#define IO_SetPinValue(GPIOx, GPIO_Pin, val)	((val) ? IO_SetPinHigh(GPIOx, GPIO_Pin) : AK_GPIO_SetPinLow(GPIOx, GPIO_Pin))

/**
 * @brief  Gets input data bit
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to read input bit value
 * @param  GPIO_Pin: GPIO pin where you want to read value
 * @retval 1 in case pin is high, or 0 if low
 */
#define IO_GetInputPinValue(GPIOx, GPIO_Pin)	(((GPIOx)->IDR & ( (1 << GPIO_Pin) )) == 0 ? 0 : 1)

/**
 * @brief  Gets output data bit
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to read output bit value
 * @param  GPIO_Pin: GPIO pin where you want to read value
 * @retval 1 in case pin is high, or 0 if low
 */
#define IO_GetOutputPinValue(GPIOx, GPIO_Pin)	(((GPIOx)->ODR & ((1 << GPIO_Pin))) == 0 ? 0 : 1)

/**
 * @brief  Gets input value from entire GPIO PORT
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to read input data value
 * @retval Entire PORT INPUT register
 */
#define IO_GetPortInputValue(GPIOx)			((GPIOx)->IDR)

/**
 * @brief  Gets output value from entire GPIO PORT
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to read output data value
 * @retval Entire PORT OUTPUT register
 */
#define IO_GetPortOutputValue(GPIOx)			((GPIOx)->ODR)

/* ------------------------------------------------------------------------------------------------------------------*/

void IO_PinInit(GPIO_TypeDef* pPort, uint32_t ClockEnable, uint32_t PinNumber, uint32_t PinMode, uint32_t PinType, uint32_t PinSpeed, uint32_t MultiState);

/* ------------------------------------------------------------------------------------------------------------------*/

#endif // _STM32F7_IO_H__
