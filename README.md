# STM32F7_FatFS_HAL_Free

BARE-METAL Example of the FatFs running on the SDMMC of the STM32F746G-DISCO No HAL at all... full bare-metal

This repository is a driver for SD card interface on the STM32F7xx processor from ST
It is base on the STM32F746G-DISCO, but it can be easily use on any project.
You only need to change the pin configuration.
The project run on EmBitz 1.1
It was originally running on Keil, so it is easy to convert back to Keil

Purpose of this project is to help other use the SDMMC module without the hassle of using the HAL (Horror At Large) library from ST.
  CON of the HAL
    - The HAL library is very hard to follow.
    - to much useless redefinition.
    - Very complicated.
    - Take a lot of space. (10K less for this driver)
    - Slow
    
 PRO of this driver.
    - Compact.
    - Ready for FatFS.
    - Easy to understand.
    - Very basic example.
    - Provide 99% of the need for any project.
    
    Please feel free to contribute. if you notice any point that may need attention.
