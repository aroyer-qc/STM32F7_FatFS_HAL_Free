#ifndef __fatfs_sd_sdio_H__
#define __fatfs_sd_sdio_H__

#include "stm32f746xx.h"
#include <stdint.h>
#include "ff.h"
#include "diskio.h"

 /* SDCARD pinouts
 *
 * SD CARD PINS
   _________________
  / 1 2 3 4 5 6 7 8 |  NR   |SDIO INTERFACE
 /                  |       |NAME     STM32F746     DESCRIPTION
/ 9                 |       |         4-BIT  1-BIT
|                   |       |
|                   |   1   |CD/DAT3  PC11   -      Connector data line 3
|                   |   2   |CMD      PD2    PD2    Command/Response line
|                   |   3   |VSS1     GND    GND    GND
|   SD CARD Pinout  |   4   |VDD      3.3V   3.3V   3.3V Power supply
|                   |   5   |CLK      PC12   PC12   Clock
|                   |   6   |VSS2     GND    GND    GND
|                   |   7   |DAT0     PC8    PC8    Connector data line 0
|                   |   8   |DAT1     PC9    -      Connector data line 1
|___________________|   9   |DAT2     PC10   -      Connector data line 2

 */

/* Define(s) --------------------------------------------------------------------------------------------------------*/

//#define MSD_OK                        		    ((uint8_t)0x00)
#define MSD_ERROR                     		    ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT      		    ((uint8_t)0x02)

#define SD_PRESENT               				((uint8_t)0x01)
#define SD_NOT_PRESENT           				((uint8_t)0x00)

#define SD_DATATIMEOUT           				((uint32_t)100000000)

#define SD_DETECT_GPIO_PORT                   	GPIOC
#define SD_DETECT_PIN             				GPIO_PIN_13

/* Structure(s) -----------------------------------------------------------------------------------------------------*/

// FATFS size structure
typedef struct
{
	uint32_t Total; // Total size of memory
	uint32_t Free;  // Free size of memory
} FATFS_Size_t;


typedef enum
{
  // SD specific error defines
    SD_CMD_CRC_FAIL                    = (1),   // Command response received (but CRC check failed)
    SD_DATA_CRC_FAIL                   = (2),   // Data block sent/received (CRC check failed)
    SD_CMD_RSP_TIMEOUT                 = (3),   // Command response TimeOut
    SD_DATA_TIMEOUT                    = (4),   // Data TimeOut
    SD_TX_UNDERRUN                     = (5),   // Transmit FIFO underrun
    SD_RX_OVERRUN                      = (6),   // Receive FIFO overrun
    SD_START_BIT_ERR                   = (7),   // Start bit not detected on all data signals in wide bus mode
    SD_CMD_OUT_OF_RANGE                = (8),   // Command's argument was out of range.
    SD_ADDR_MISALIGNED                 = (9),   // Misaligned address
    SD_BLOCK_LEN_ERR                   = (10),  // Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length
    SD_ERASE_SEQ_ERR                   = (11),  // An error in the sequence of erase command occurs.
    SD_BAD_ERASE_PARAM                 = (12),  // An invalid selection for erase groups
    SD_WRITE_PROT_VIOLATION            = (13),  // Attempt to program a write protect block
    SD_LOCK_UNLOCK_FAILED              = (14),  // Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card
    SD_COM_CRC_FAILED                  = (15),  // CRC check of the previous command failed
    SD_ILLEGAL_CMD                     = (16),  // Command is not legal for the card state
    SD_CARD_ECC_FAILED                 = (17),  // Card internal ECC was applied but failed to correct the data
    SD_CC_ERROR                        = (18),  // Internal card controller error
    SD_GENERAL_UNKNOWN_ERROR           = (19),  // General or unknown error
    SD_STREAM_READ_UNDERRUN            = (20),  // The card could not sustain data transfer in stream read operation.
    SD_STREAM_WRITE_OVERRUN            = (21),  // The card could not sustain data programming in stream mode
    SD_CID_CSD_OVERWRITE               = (22),  // CID/CSD overwrite error
    SD_WP_ERASE_SKIP                   = (23),  // Only partial address space was erased
    SD_CARD_ECC_DISABLED               = (24),  // Command has been executed without using internal ECC
    SD_ERASE_RESET                     = (25),  // Erase sequence was cleared before executing because an out of erase sequence command was received
    SD_AKE_SEQ_ERROR                   = (26),  // Error in sequence of authentication.
    SD_INVALID_VOLTRANGE               = (27),
    SD_ADDR_OUT_OF_RANGE               = (28),
    SD_SWITCH_ERROR                    = (29),
    SD_SDMMC_DISABLED                  = (30),
    SD_SDMMC_FUNCTION_BUSY             = (31),
    SD_SDMMC_FUNCTION_FAILED           = (32),
    SD_SDMMC_UNKNOWN_FUNCTION          = (33),

    // Standard error defines
    SD_INTERNAL_ERROR                  = (34),
    SD_NOT_CONFIGURED                  = (35),
    SD_REQUEST_PENDING                 = (36),
    SD_REQUEST_NOT_APPLICABLE          = (37),
    SD_INVALID_PARAMETER               = (38),
    SD_UNSUPPORTED_FEATURE             = (39),
    SD_UNSUPPORTED_HW                  = (40),
    SD_ERROR                           = (41),
    SD_BUSY                            = (42),
    SD_OK                              = (0)
} SD_Error_t;


typedef struct
{
    uint8_t  DAT_BUS_WIDTH;           // Shows the currently defined data bus width
    uint8_t  SECURED_MODE;            // Card is in secured mode of operation
    uint16_t SD_CARD_TYPE;            // Carries information about card type
    uint32_t SIZE_OF_PROTECTED_AREA;  // Carries information about the capacity of protected area
    uint8_t  SPEED_CLASS;             // Carries information about the speed class of the card
    uint8_t  PERFORMANCE_MOVE;        // Carries information about the card's performance move
    uint8_t  AU_SIZE;                 // Carries information about the card's allocation unit size
    uint16_t ERASE_SIZE;              // Determines the number of AUs to be erased in one operation
    uint8_t  ERASE_TIMEOUT;           // Determines the TimeOut for any number of AU erase
    uint8_t  ERASE_OFFSET;            // Carries information about the erase offset
} SD_CardStatus_t;


/* Prototype(s) -----------------------------------------------------------------------------------------------------*/

void    		 SD_Initialize               (void);
DSTATUS          FATFS_SD_disk_initialize    (void);
DSTATUS          FATFS_SD_disk_status        (void);
DRESULT          FATFS_SD_disk_ioctl         (BYTE cmd, void *buff);
DRESULT          FATFS_SD_disk_read          (BYTE *buff, DWORD sector, UINT count);
DRESULT          FATFS_SD_disk_write         (const BYTE *buff, DWORD sector, UINT count);
FRESULT          FATFS_GetDriveSize          (char* str, FATFS_Size_t* SizeStruct);

SD_Error_t       SD_GetStatus                (void);
SD_Error_t       SD_GetCardInfo              (void);

SD_Error_t       SD_Erase                    (uint64_t startaddr, uint64_t endaddr);
SD_Error_t       SD_SendSDStatus             (uint32_t *pSDstatus);
SD_Error_t       SD_GetCardStatus            (SD_CardStatus_t* pCardStatus);

/* ------------------------------------------------------------------------------------------------------------------*/

#endif // __fatfs_sd_sdio_H__
