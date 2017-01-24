/* Includes ---------------------------------------------------------------------------------------------------------*/

#include "bsp.h"
#include "stm32f746xx.h"
#include "fatfs_sd_sdio.h"

/* Defines ----------------------------------------------------------------------------------------------------------*/

#define NVIC_PRIORITYGROUP_4            ((uint32_t)0x00000003)

// PLL Parameters

// PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N
#define PLL_M      25
#define PLL_N      432

// SYSCLK = PLL_VCO / PLL_P
#define PLL_P      2
// USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ
#define PLL_Q      9
#define PWR_OVERDRIVE_TIMEOUT_VALUE     1000
#define VECT_TAB_OFFSET                 0x00

/* Variables --------------------------------------------------------------------------------------------------------*/

uint32_t Tick;
uint32_t SystemCoreClock;

/* ------------------------------------------------------------------------------------------------------------------*/

/**		Initialize miscellaneous item or module
  *
  * @brief  Initialize LED and system tick
  */
void Initialize(void)
{
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    // Systick
    SysTick->LOAD  =  (SystemCoreClock / 1000) - 1;                      // Set reload register to IRC
    NVIC_SetPriority (SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);   // Set Priority for Systick Interrupt
    SysTick->VAL   = 0;                                             // Load the SysTick Counter Value
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;                       // Enable SysTick IRQ and SysTick Timer

	IO_PinInit(LED_PORT, RCC_AHB1ENR_GPIOIEN, LED1, GPIO_MODER_OUTPUT, GPIO_TYPE_PIN_PP, GPIO_OSPEEDR_LOW, 0);
	SD_Initialize();
}

/** -----------------------------------------------------------------------------------------------------------------*/
/**		System Clock Configuration

  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  */
void SystemInit(void)
{
    // FPU settings
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR  |= ((3UL << 10 * 2) | (3UL << 11 * 2));     // set CP10 and CP11 Full Access
  #endif

    // Reset the RCC clock configuration to the default reset state
    RCC->CR      |= (uint32_t)0x00000001;                                   // Set HSION bit
    RCC->CFGR     = 0x00000000;                                             // Reset CFGR register
    RCC->CR      &= (uint32_t)0xFEF6FFFF;                                   // Reset HSEON, CSSON and PLLON bits
    RCC->PLLCFGR  = 0x24003010;                                             // Reset PLLCFGR register
    RCC->CR      &= (uint32_t)0xFFFBFFFF;                                   // Reset HSEBYP bit
    RCC->CIR      = 0x00000000;                                             // Disable all interrupts


    //----------------------------- HSE Configuration --------------------------
    // Enable the External High Speed oscillator (HSE).
    RCC->CR |= RCC_CR_HSEON;

    // Wait till HSE is ready */
    while((RCC->CR & RCC_CR_HSERDY) == 0);

    //-------------------------------- Power Control ---------------------------

    // Enable Power Control clock
    RCC->APB1ENR = RCC_APB1ENR_PWREN;

    // The voltage scaling allows optimizing the power consumption when the device is
    // clocked below the maximum system frequency, to update the voltage scaling value
    // regarding system frequency refer to product datasheet.
    //MODIFY_REG(PWR->CR1, PWR_CR1_VOS, PWR_REGULATOR_VOLTAGE_SCALE1);

    //-------------------------------- PLL Configuration -----------------------

    // Disable the main PLL.
    RCC->CR &= ~RCC_CR_PLLON;

    // Configure the main PLL clock source, multiplication and division factors.
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

    RCC->CR |= RCC_CR_PLLON;                                    // Enable the main PLL.
    while((RCC->CR & RCC_CR_PLLRDY) == 0);                      // Wait till PLL is ready
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_7WS;      // Configure Flash prefetch and wait state
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));          // Select the main PLL as system clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);      // Wait till the main PLL is used as system clock source
    SystemCoreClock = (PLL_N / PLL_P) * 1000000UL;

    // AHB CLOCK
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                // Set to 216 MHz
    // APB1 CLOCK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;               // Set to 54 MHz
    // APB2 CLOCK
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;               // set to 108 MHz

    RCC->CR |= RCC_CR_PLLON;                                                                        // Enable the main PLL
    while((RCC->CR & RCC_CR_PLLRDY) == 0);                                                          // Wait till the main PLL is ready


    // Enable the Over-drive to extend the clock frequency to 216 MHz
    PWR->CR1 |= (uint32_t)PWR_CR1_ODEN;
    while((PWR->CSR1 & PWR_CSR1_ODRDY) != 0){};
    PWR->CR1 |= (uint32_t)PWR_CR1_ODSWEN;           // Enable the Over-drive switch
    while((PWR->CSR1 & PWR_CSR1_ODSWRDY) != 0){};

    // Configure the Vector Table location add offset address
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;       // Vector Table Relocation in Internal FLASH
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**		SysTick IRQ Handler
  *
  * @brief  Increment the system tick
  */

void SysTick_Handler(void)
{
    Tick++;
}

/* ------------------------------------------------------------------------------------------------------------------*/
