/* Include(s) -------------------------------------------------------------------------------------------------------*/

#include "stdbool.h"
#include <string.h>
#include "stm32f7xx.h"
#include "stm32f746xx.h"
#include "fatfs_sd_sdio.h"
#include "io.h"
#include "bsp.h"

/* Define(s) --------------------------------------------------------------------------------------------------------*/

#define DMA_CHANNEL_4                   ((uint32_t)0x08000000)
#define DMA_MEMORY_TO_PERIPH            ((uint32_t)DMA_SxCR_DIR_0)
#define DMA_MINC_ENABLE                 ((uint32_t)DMA_SxCR_MINC)
#define DMA_MDATAALIGN_WORD             ((uint32_t)DMA_SxCR_MSIZE_1)
#define DMA_PDATAALIGN_WORD             ((uint32_t)DMA_SxCR_PSIZE_1)
#define DMA_PRIORITY_VERY_HIGH          ((uint32_t)DMA_SxCR_PL)
#define DMA_MBURST_INC4                 ((uint32_t)DMA_SxCR_MBURST_0)
#define DMA_PBURST_INC4                 ((uint32_t)DMA_SxCR_PBURST_0)

#define BLOCK_SIZE                      ((uint32_t)(512))

#define IFCR_CLEAR_MASK_STREAM3         (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3)
#define IFCR_CLEAR_MASK_STREAM6         (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6)

#define SDMMC_ICR_STATIC_FLAGS          ((uint32_t)(SDMMC_ICR_CCRCFAILC | SDMMC_ICR_DCRCFAILC | SDMMC_ICR_CTIMEOUTC |\
                                                    SDMMC_ICR_DTIMEOUTC | SDMMC_ICR_TXUNDERRC | SDMMC_ICR_RXOVERRC  |\
                                                    SDMMC_ICR_CMDRENDC  | SDMMC_ICR_CMDSENTC  | SDMMC_ICR_DATAENDC  |\
                                                    SDMMC_ICR_DBCKENDC))

#define SDMMC_CMD0TIMEOUT               ((uint32_t)0x00010000)

#define SD_OCR_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000)
#define SD_OCR_ADDR_MISALIGNED          ((uint32_t)0x40000000)
#define SD_OCR_BLOCK_LEN_ERR            ((uint32_t)0x20000000)
#define SD_OCR_ERASE_SEQ_ERR            ((uint32_t)0x10000000)
#define SD_OCR_BAD_ERASE_PARAM          ((uint32_t)0x08000000)
#define SD_OCR_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000)
#define SD_OCR_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000)
#define SD_OCR_COM_CRC_FAILED           ((uint32_t)0x00800000)
#define SD_OCR_ILLEGAL_CMD              ((uint32_t)0x00400000)
#define SD_OCR_CARD_ECC_FAILED          ((uint32_t)0x00200000)
#define SD_OCR_CC_ERROR                 ((uint32_t)0x00100000)
#define SD_OCR_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000)
#define SD_OCR_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000)
#define SD_OCR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000)
#define SD_OCR_CID_CSD_OVERWRITE        ((uint32_t)0x00010000)
#define SD_OCR_WP_ERASE_SKIP            ((uint32_t)0x00008000)
#define SD_OCR_CARD_ECC_DISABLED        ((uint32_t)0x00004000)
#define SD_OCR_ERASE_RESET              ((uint32_t)0x00002000)
#define SD_OCR_AKE_SEQ_ERROR            ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS                ((uint32_t)0xFDFFE008)

#define SD_R6_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00002000)
#define SD_R6_ILLEGAL_CMD               ((uint32_t)0x00004000)
#define SD_R6_COM_CRC_FAILED            ((uint32_t)0x00008000)

#define SD_VOLTAGE_WINDOW_SD            ((uint32_t)0x80100000)
#define SD_HIGH_CAPACITY                ((uint32_t)0x40000000)
#define SD_STD_CAPACITY                 ((uint32_t)0x00000000)
#define SD_CHECK_PATTERN                ((uint32_t)0x000001AA)

#define SD_MAX_VOLT_TRIAL               ((uint32_t)0x0000FFFF)
#define SD_ALLZERO                      ((uint32_t)0x00000000)

#define SD_WIDE_BUS_SUPPORT             ((uint32_t)0x00040000)
#define SD_SINGLE_BUS_SUPPORT           ((uint32_t)0x00010000)
#define SD_CARD_LOCKED                  ((uint32_t)0x02000000)

#define SD_0TO7BITS                     ((uint32_t)0x000000FF)
#define SD_8TO15BITS                    ((uint32_t)0x0000FF00)
#define SD_16TO23BITS                   ((uint32_t)0x00FF0000)
#define SD_24TO31BITS                   ((uint32_t)0xFF000000)
#define SD_MAX_DATA_LENGTH              ((uint32_t)0x01FFFFFF)

#define SD_CCCC_ERASE                   ((uint32_t)0x00000020)

#define SD_SDMMC_SEND_IF_COND           ((uint32_t)SD_CMD_HS_SEND_EXT_CSD)



#define SDMMC_BUS_WIDE_1B               ((uint32_t)0x00000000)
#define SDMMC_BUS_WIDE_4B               SDMMC_CLKCR_WIDBUS_0
#define SDMMC_BUS_WIDE_8B               SDMMC_CLKCR_WIDBUS_1

#define SDMMC_CMD_RESPONSE_SHORT        SDMMC_CMD_WAITRESP_0
#define SDMMC_CMD_RESPONSE_LONG         SDMMC_CMD_WAITRESP

#define SDMMC_DATABLOCK_SIZE_8B         (SDMMC_DCTRL_DBLOCKSIZE_0|SDMMC_DCTRL_DBLOCKSIZE_1)
#define SDMMC_DATABLOCK_SIZE_64B        (SDMMC_DCTRL_DBLOCKSIZE_1|SDMMC_DCTRL_DBLOCKSIZE_2)
#define SDMMC_DATABLOCK_SIZE_512B       (SDMMC_DCTRL_DBLOCKSIZE_0|SDMMC_DCTRL_DBLOCKSIZE_3)

#define CLKCR_CLEAR_MASK                ((uint32_t)(SDMMC_CLKCR_CLKDIV  | SDMMC_CLKCR_PWRSAV |\
                                                    SDMMC_CLKCR_BYPASS  | SDMMC_CLKCR_WIDBUS |\
                                                    SDMMC_CLKCR_NEGEDGE | SDMMC_CLKCR_HWFC_EN))

#define DCTRL_CLEAR_MASK                ((uint32_t)(SDMMC_DCTRL_DTEN    | SDMMC_DCTRL_DTDIR |\
                                                    SDMMC_DCTRL_DTMODE  | SDMMC_DCTRL_DBLOCKSIZE))

#define CMD_CLEAR_MASK                  ((uint32_t)(SDMMC_CMD_CMDINDEX | SDMMC_CMD_WAITRESP |\
                                                    SDMMC_CMD_WAITINT  | SDMMC_CMD_WAITPEND |\
                                                    SDMMC_CMD_CPSMEN   | SDMMC_CMD_SDIOSUSPEND))

#define SDMMC_INIT_CLK_DIV              ((uint8_t)0x76)


#define STD_CAPACITY_SD_CARD_V1_1       ((uint32_t)0x00000000)
#define STD_CAPACITY_SD_CARD_V2_0       ((uint32_t)0x00000001)
#define HIGH_CAPACITY_SD_CARD           ((uint32_t)0x00000002)
#define MULTIMEDIA_CARD                 ((uint32_t)0x00000003)
#define SECURE_DIGITAL_IO_CARD          ((uint32_t)0x00000004)
#define HIGH_SPEED_MULTIMEDIA_CARD      ((uint32_t)0x00000005)
#define SECURE_DIGITAL_IO_COMBO_CARD    ((uint32_t)0x00000006)
#define HIGH_CAPACITY_MMC_CARD          ((uint32_t)0x00000007)

#define SD_CMD_GO_IDLE_STATE            ((uint8_t)0)   // Resets the SD memory card.
#define SD_CMD_SEND_OP_COND             ((uint8_t)1)   // Sends host capacity support information and activates the card's initialization process.
#define SD_CMD_ALL_SEND_CID             ((uint8_t)2)   // Asks any card connected to the host to send the CID numbers on the CMD line.
#define SD_CMD_SET_REL_ADDR             ((uint8_t)3)   // Asks the card to publish a new relative address (RCA).
#define SD_CMD_HS_SWITCH                ((uint8_t)6)   // Checks switchable function (mode 0) and switch card function (mode 1).
#define SD_CMD_SEL_DESEL_CARD           ((uint8_t)7)   // Selects the card by its own relative address and gets deselected by any other address
#define SD_CMD_HS_SEND_EXT_CSD          ((uint8_t)8)   // Sends SD Memory Card interface condition, which includes host supply voltage information
                                                       // and asks the card whether card supports voltage.
#define SD_CMD_SEND_CSD                 ((uint8_t)9)   // Addressed card sends its card specific data (CSD) on the CMD line.
#define SD_CMD_SEND_CID                 ((uint8_t)10)  // Addressed card sends its card identification (CID) on the CMD line.
#define SD_CMD_STOP_TRANSMISSION        ((uint8_t)12)  // Forces the card to stop transmission.
#define SD_CMD_SEND_STATUS              ((uint8_t)13)  // Addressed card sends its status register.
#define SD_CMD_SET_BLOCKLEN             ((uint8_t)16)  // Sets the block length (in bytes for SDSC) for all following block commands
                                                       // (read, write, lock). Default block length is fixed to 512 Bytes. Not effective
                                                       // for SDHS and SDXC.
#define SD_CMD_READ_SINGLE_BLOCK        ((uint8_t)17)  // Reads single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of
                                                       // fixed 512 bytes in case of SDHC and SDXC.
#define SD_CMD_READ_MULT_BLOCK          ((uint8_t)18)  // Continuously transfers data blocks from card to host until interrupted by
                                                       // STOP_TRANSMISSION command.
#define SD_CMD_WRITE_SINGLE_BLOCK       ((uint8_t)24)  // Writes single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of
                                                       // fixed 512 bytes in case of SDHC and SDXC.
#define SD_CMD_WRITE_MULT_BLOCK         ((uint8_t)25)  // Continuously writes blocks of data until a STOP_TRANSMISSION follows.
#define SD_CMD_SD_ERASE_GRP_START       ((uint8_t)32)  // Sets the address of the first write block to be erased. (For SD card only).
#define SD_CMD_SD_ERASE_GRP_END         ((uint8_t)33)  // Sets the address of the last write block of the continuous range to be erased.
                                                       // system set by switch function command (CMD6).
#define SD_CMD_ERASE                    ((uint8_t)38)  // Reserved for SD security applications.
#define SD_CMD_FAST_IO                  ((uint8_t)39)  // SD card doesn't support it (Reserved).
#define SD_CMD_APP_CMD                  ((uint8_t)55)  // Indicates to the card that the next command is an application specific command rather
                                                       // than a standard command.

// Following commands are SD Card Specific commands.
// SDMMC_APP_CMD should be sent before sending these commands.
#define SD_CMD_APP_SD_SET_BUSWIDTH      ((uint8_t)6)   // (ACMD6) Defines the data bus width to be used for data transfer. The allowed data bus
                                                       // widths are given in SCR register.
#define SD_CMD_SD_APP_STATUS            ((uint8_t)13)  // (ACMD13) Sends the SD status.
#define SD_CMD_SD_APP_OP_COND           ((uint8_t)41)  // (ACMD41) Sends host capacity support information (HCS) and asks the accessed card to
                                                       // send its operating condition register (OCR) content in the response on the CMD line.
#define SD_CMD_SD_APP_SEND_SCR          ((uint8_t)51)  // Reads the SD Configuration Register (SCR).



/* Typedef(s) -------------------------------------------------------------------------------------------------------*/

typedef enum
{
    HAL_OK       = 0x00,
    HAL_ERROR    = 0x01,
    HAL_BUSY     = 0x02,
    HAL_TIMEOUT  = 0x03
} StatusTypeDef;


typedef enum
{
    SD_SINGLE_BLOCK    = 0,       // Single block operation
    SD_MULTIPLE_BLOCK  = 1,       // Multiple blocks operation
} SD_Operation_t;

typedef struct
{
  volatile SD_CSDTypedef    SD_csd;         // SD card specific data register
  volatile SD_CIDTypedef    SD_cid;         // SD card identification number register
  uint64_t                  CardCapacity;   // Card capacity
  uint32_t                  CardBlockSize;  // Card block size
  uint16_t                  RCA;            // SD relative card address
  uint8_t                   CardType;       // SD card type

} SD_CardInfo_t;


/* Variable(s) ------------------------------------------------------------------------------------------------------*/

static SD_HandleTypeDef uSdHandle;
static SD_CardInfo_t    SD_CardInfo;
static DSTATUS          Stat;

/* Private function(s) ----------------------------------------------------------------------------------------------*/

static bool 	        SD_IsDetected           (void);
static void             SD_GetResponse          (uint32_t* pResponse);
static void             SD_DMA_RxCplt           (void);
static void             SD_DMA_TxCplt           (void);
static StatusTypeDef    SD_DMA_Abort            (DMA_Stream_TypeDef* pDMA_Stream);
static SD_Error_t  HAL_SD_Init             (void);
static SD_Error_t  SD_InitializeCard       (void);
static uint8_t 	        SD_ReadBlocks_DMA       (uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
static uint8_t 	        SD_WriteBlocks_DMA      (uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
static SD_Error_t  SD_CheckWriteOperation  (uint32_t Timeout);
static SD_Error_t  SD_CheckReadOperation   (uint32_t Timeout);


static SD_Error_t  SD_Select_Deselect      (uint64_t addr);
static SD_Error_t  SD_PowerON              (void);
static void             SD_PowerOFF             (void);
static SD_Error_t  SD_SendStatus           (uint32_t *pCardStatus);
static SD_Error_t  SD_IsCardProgramming    (uint8_t *pStatus);
static SD_Error_t  SD_CmdError             (void);
static SD_Error_t  SD_CmdResp1Error        (uint8_t SD_CMD);
static SD_Error_t  SD_CmdResp7Error        (void);
static SD_Error_t  SD_CmdResp3Error        (void);
static SD_Error_t  SD_CmdResp2Error        (void);
static SD_Error_t  SD_CmdResp6Error        (uint8_t SD_CMD, uint16_t *pRCA);
static SD_Error_t  SD_WideBusEnable        (void);
static SD_Error_t  SD_WideBusDisable       (void);
static SD_Error_t  SD_FindSCR              (uint32_t *pSCR);



/** -----------------------------------------------------------------------------------------------------------------*/
/**		SD_IsDetected
  *
  * @brief  Test if card is present
  * @param  uint8_t       true or false
  */
static bool SD_IsDetected(void)
{
    return (IO_GetInputPinValue(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) == 0);
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**		GetResponse
  *
  * @brief  Get response from SD device
  * @param  uint32_t*       pResponse
  */
static void SD_GetResponse(uint32_t* pResponse)
{
    if(pResponse != NULL)
    {
        pResponse[0] = SDMMC1->RESP1;
        pResponse[1] = SDMMC1->RESP2;
        pResponse[2] = SDMMC1->RESP3;
        pResponse[3] = SDMMC1->RESP4;
    }
}


/** -----------------------------------------------------------------------------------------------------------------*/
static StatusTypeDef SD_DMA_Abort(DMA_Stream_TypeDef* pDMA_Stream)
{
    uint32_t tickstart;

    // Disable the stream
    pDMA_Stream->CR &= ~DMA_SxCR_EN;

    // Get tick
    tickstart = Tick;

    // Check if the DMA Stream is effectively disabled
    tickstart = 0;
    while((pDMA_Stream->CR & DMA_SxCR_EN) != 0)
    {
        if((Tick - tickstart ) > 1000)      // Check for the Timeout
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  SD DMA transfer complete Rx callback.
  */
static void SD_DMA_RxCplt(void)
{
    uSdHandle.DmaTransferCplt = 1;          // DMA transfer is complete
    while(uSdHandle.SdTransferCplt == 0){}; // Wait until SD transfer is complete
    SD_DMA_Abort(DMA2_Stream3);             // Disable the DMA channel
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  SD DMA transfer complete Tx callback.
  */
static void SD_DMA_TxCplt(void)
{
    uSdHandle.DmaTransferCplt = 1;          // DMA transfer is complete
    while(uSdHandle.SdTransferCplt == 0){}; // Wait until SD transfer is complete
    SD_DMA_Abort(DMA2_Stream6);             // Disable the DMA channel
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Initializes the SD card.
  * @retval HAL SD error state
  */
SD_Error_t HAL_SD_Init(void)
{
    SD_Error_t errorstate = SD_OK;

    // Initialize SDMMC peripheral interface with default configuration for SD card initialization
    SDMMC1->CLKCR &= ~(uint32_t)CLKCR_CLEAR_MASK;
    SDMMC1->CLKCR |=  (uint32_t)SDMMC_INIT_CLK_DIV;

    // Identify card operating voltage
    errorstate = SD_PowerON();

    if(errorstate != SD_OK)
    {
        return errorstate;
    }

    // Initialize the present card and put them in idle state
    errorstate = SD_InitializeCard();

    if(errorstate != SD_OK)
    {
        return errorstate;
    }

    // Read CSD/CID MSD registers
    errorstate = SD_GetCardInfo();

    if(errorstate == SD_OK)
    {
        // Select the Card
        errorstate = SD_Select_Deselect((uint32_t)(((uint32_t)SD_CardInfo.RCA) << 16));
    }

    // Configure SDMMC peripheral interface
    SDMMC1->CLKCR &= ~(uint32_t)CLKCR_CLEAR_MASK;

    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Initializes all cards or single card as the case may be Card(s) come
  *         into standby state.
  * @retval SD Card error state
  */
static SD_Error_t SD_InitializeCard(void)
{
    SD_Error_t errorstate = SD_OK;
    uint16_t sd_rca = 1;

    if((SDMMC1->POWER & SDMMC_POWER_PWRCTRL) == 0) // Power off
    {
        return SD_REQUEST_NOT_APPLICABLE;
    }

    if(uSdHandle.CardType != SECURE_DIGITAL_IO_CARD)
    {
        // Send CMD2 ALL_SEND_CID
        SDMMC1->ARG  =  (uint32_t)0;
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_ALL_SEND_CID | SDMMC_CMD_RESPONSE_LONG | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        if((errorstate = SD_CmdResp2Error()) != SD_OK)
        {
            return errorstate;
        }

        // Get Card identification number data
        SD_GetResponse(uSdHandle.CID);
    }

    if((uSdHandle.CardType == STD_CAPACITY_SD_CARD_V1_1)    || (uSdHandle.CardType == STD_CAPACITY_SD_CARD_V2_0) ||
       (uSdHandle.CardType == SECURE_DIGITAL_IO_COMBO_CARD) || (uSdHandle.CardType == HIGH_CAPACITY_SD_CARD))
    {
        // Send CMD3 SET_REL_ADDR with argument 0
        // SD Card publishes its RCA.
        SDMMC1->ARG  =  (uint32_t)0;
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_SET_REL_ADDR | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        if((errorstate = SD_CmdResp6Error( SD_CMD_SET_REL_ADDR, &sd_rca)) != SD_OK)
        {
            return errorstate;
        }
    }

    if(uSdHandle.CardType != SECURE_DIGITAL_IO_CARD)
    {
        // Get the SD card RCA
        uSdHandle.RCA = sd_rca;

        // Send CMD9 SEND_CSD with argument as card's RCA
        SDMMC1->ARG  =  (uint32_t)(uint32_t)(uSdHandle.RCA << 16);
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_SEND_CSD | SDMMC_CMD_RESPONSE_LONG | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        if((errorstate = SD_CmdResp2Error()) == SD_OK)
        {
            // Get Card Specific Data
            SD_GetResponse(uSdHandle.CSD);
        }
    }

    // Card are initialized
    return errorstate;
}

/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   This API should be followed by the function SD_CheckReadOperation()
  *         to check the completion of the read process
  * @param  pReadBuffer: Pointer to the buffer that will contain the received data
  * @param  ReadAddr: Address from where data is to be read
  * @param  BlockSize: SD card Data block size
  * @note   BlockSize must be 512 bytes.
  * @param  NumberOfBlocks: Number of blocks to read.
  * @retval SD Card error state
  */
SD_Error_t SD_ReadBlocks_DMA(uint32_t *pReadBuffer, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t CmdIndex;

    SDMMC1->DCTRL = 0;                  // Initialize data control register
    uSdHandle.SdTransferCplt  = 0;      // Initialize handle flags
    uSdHandle.DmaTransferCplt = 0;
    uSdHandle.SdTransferErr   = SD_OK;

    // Initialize SD Read operation
    uSdHandle.SdOperation = (NumberOfBlocks > 1) ? SD_MULTIPLE_BLOCK : SD_SINGLE_BLOCK;

    // Enable transfer interrupts
    SDMMC1->MASK |= (SDMMC_MASK_DCRCFAILIE | SDMMC_MASK_DTIMEOUTIE | SDMMC_MASK_DATAENDIE | SDMMC_MASK_RXOVERRIE);

    // Enable the DMA Channel
    SDMMC1->DCTRL |= SDMMC_DCTRL_DMAEN;                                                     // Enable SDMMC DMA transfer
    DMA2_Stream3->CR  &= ~DMA_SxCR_EN;                                                      // Disable the Peripheral
    DMA2_Stream3->NDTR = (uint32_t)(BlockSize * NumberOfBlocks) / 4;                        // Configure DMA Stream data length
    DMA2_Stream3->M0AR = (uint32_t)pReadBuffer;                                             // Configure DMA Stream destination address
    DMA2_Stream3->CR  |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;    // Enable all interrupts
    DMA2_Stream3->FCR |= DMA_SxFCR_FEIE;
    DMA2_Stream3->CR  |= DMA_SxCR_EN;                                                       // Enable the Peripheral

    if(uSdHandle.CardType == HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = 512;
        ReadAddr /= 512;
    }

    // Set Block Size for Card
    SDMMC1->ARG  =  (uint32_t)BlockSize;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_SET_BLOCKLEN | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((errorstate = SD_CmdResp1Error(SD_CMD_SET_BLOCKLEN)) != SD_OK)
    {
        return errorstate;
    }

    // Configure the SD DPSM (Data Path State Machine)
    SDMMC1->DTIMER = SD_DATATIMEOUT;
    SDMMC1->DLEN   = BlockSize * NumberOfBlocks;
    SDMMC1->DCTRL &= ~(uint32_t)DCTRL_CLEAR_MASK;
    SDMMC1->DCTRL |=  (uint32_t)(SDMMC_DATABLOCK_SIZE_512B | SDMMC_DCTRL_DTDIR | SDMMC_DCTRL_DTEN);

    // Send CMD18 READ_MULT_BLOCK with argument data address
    // or send CMD17 READ_SINGLE_BLOCK depending on number of block
    CmdIndex = (NumberOfBlocks > 1) ?SD_CMD_READ_MULT_BLOCK : SD_CMD_READ_SINGLE_BLOCK;

    SDMMC1->ARG  =  (uint32_t)ReadAddr;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(CmdIndex | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    errorstate = SD_CmdResp1Error((NumberOfBlocks > 1) ? SD_CMD_READ_MULT_BLOCK : SD_CMD_READ_SINGLE_BLOCK);

    // Update the SD transfer error in SD handle
    uSdHandle.SdTransferErr = errorstate;

    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Writes block(s) to a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   This API should be followed by the function HAL_SD_CheckWriteOperation()
  *         to check the completion of the write process (by SD current status polling).
  * @param  pWriteBuffer: pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be read
  * @param  BlockSize: the SD card Data block size
  * @note   BlockSize must be 512 bytes.
  * @param  NumberOfBlocks: Number of blocks to write
  * @retval SD Card error state
  */
SD_Error_t SD_WriteBlocks_DMA(uint32_t *pWriteBuffer, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t        CmdIndex;

    SDMMC1->DCTRL = 0;              // Initialize data control register
    uSdHandle.SdTransferCplt  = 0;  // Initialize handle flags
    uSdHandle.DmaTransferCplt = 0;
    uSdHandle.SdTransferErr   = SD_OK;

    // Initialize SD Write operation
    uSdHandle.SdOperation = (NumberOfBlocks > 1) ? SD_MULTIPLE_BLOCK : SD_SINGLE_BLOCK;

    // Enable transfer interrupts
    SDMMC1->MASK |= (SDMMC_MASK_DCRCFAILIE | SDMMC_MASK_DTIMEOUTIE | SDMMC_MASK_DATAENDIE | SDMMC_MASK_TXUNDERRIE);

    // Enable the DMA Channel
    SDMMC1->DCTRL |= SDMMC_DCTRL_DMAEN;                                                     // Enable SDMMC DMA transfer
    DMA2_Stream6->CR  &= ~DMA_SxCR_EN;                                                      // Disable the Peripheral
    DMA2_Stream6->NDTR = (uint32_t)(BlockSize * NumberOfBlocks) / 4;                        // Configure DMA Stream data length
    DMA2_Stream6->M0AR = (uint32_t)pWriteBuffer;                                            // Configure DMA Stream destination address
    DMA2_Stream6->CR  |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;    // Enable all interrupts
    DMA2_Stream6->FCR |= DMA_SxFCR_FEIE;
    DMA2_Stream6->CR  |= DMA_SxCR_EN;                                                       // Enable the Peripheral

    // Enable SDMMC DMA transfer
    SDMMC1->DCTRL |= SDMMC_DCTRL_DMAEN;

    if(uSdHandle.CardType == HIGH_CAPACITY_SD_CARD)
    {
        BlockSize  = 512;
        WriteAddr /= 512;
    }

    // Set Block Size for Card
    SDMMC1->ARG  =  (uint32_t)BlockSize;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_SET_BLOCKLEN | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((errorstate = SD_CmdResp1Error( SD_CMD_SET_BLOCKLEN)) != SD_OK)
    {
        return errorstate;
    }

    // Check number of blocks command
    // Send CMD24 WRITE_SINGLE_BLOCK
    // Send CMD25 WRITE_MULT_BLOCK with argument data address
    CmdIndex = (NumberOfBlocks > 1) ? SD_CMD_WRITE_MULT_BLOCK : SD_CMD_WRITE_SINGLE_BLOCK;

    // Set Block Size for Card
    SDMMC1->ARG  =  (uint32_t)WriteAddr;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(CmdIndex | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((SD_CmdResp1Error((NumberOfBlocks > 1) ? SD_CMD_WRITE_MULT_BLOCK : SD_CMD_WRITE_SINGLE_BLOCK)) != SD_OK)
    {
        return errorstate;
    }

    // Configure the SD DPSM (Data Path State Machine)
    SDMMC1->DTIMER = SD_DATATIMEOUT;
    SDMMC1->DLEN   = (uint32_t)(BlockSize * NumberOfBlocks);
    SDMMC1->DCTRL &= ~(uint32_t)DCTRL_CLEAR_MASK;
    SDMMC1->DCTRL |=  (uint32_t)(SDMMC_DATABLOCK_SIZE_512B | SDMMC_DCTRL_DTEN);

    uSdHandle.SdTransferErr = errorstate;

    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  This function waits until the SD DMA data read transfer is finished.
  *         This API should be called after SD_ReadBlocks_DMA() function
  *         to insure that all data sent by the card is already transferred by the
  *         DMA controller.
  * @param  Timeout: Timeout duration
  * @retval SD Card error state
  */
SD_Error_t SD_CheckReadOperation(uint32_t Timeout)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t timeout = Timeout;
    uint32_t tmp1, tmp2;
    SD_Error_t tmp3;

    // Wait for DMA/SD transfer end or SD error variables to be in SD handle
    tmp1 = uSdHandle.DmaTransferCplt;
    tmp2 = uSdHandle.SdTransferCplt;
    tmp3 = (SD_Error_t)uSdHandle.SdTransferErr;

    while(((tmp1 & tmp2) == 0) && (tmp3 == SD_OK) && (timeout > 0))
    {
        tmp1 = uSdHandle.DmaTransferCplt;
        tmp2 = uSdHandle.SdTransferCplt;
        tmp3 = (SD_Error_t)uSdHandle.SdTransferErr;
        timeout--;
    }

    timeout = Timeout;

    // Wait until the Rx transfer is no longer active
    while(((SDMMC1->STA & SDMMC_STA_RXACT) != 0) && (timeout > 0))
    {
        timeout--;
    }

    // Send stop command in multi block read
    if(uSdHandle.SdOperation == SD_MULTIPLE_BLOCK)
    {
        errorstate = SD_StopTransfer();
    }

    if((timeout == 0) && (errorstate == SD_OK))
    {
        errorstate = SD_DATA_TIMEOUT;
    }

    // Clear all the static flags
    SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;

    // Return error state
    if(uSdHandle.SdTransferErr != SD_OK)
    {
        return (SD_Error_t)(uSdHandle.SdTransferErr);
    }

    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  This function waits until the SD DMA data write transfer is finished.
  *         This API should be called after SD_WriteBlocks_DMA() function
  *         to insure that all data sent by the card is already transferred by the
  *         DMA controller.
  * @param  Timeout: Timeout duration
  * @retval SD Card error state
  */
SD_Error_t SD_CheckWriteOperation(uint32_t Timeout)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t timeout = Timeout;
    uint32_t tmp1, tmp2;
    SD_Error_t tmp3;

    // Wait for DMA/SD transfer end or SD error variables to be in SD handle
    tmp1 = uSdHandle.DmaTransferCplt;
    tmp2 = uSdHandle.SdTransferCplt;
    tmp3 = (SD_Error_t)uSdHandle.SdTransferErr;

    while(((tmp1 & tmp2) == 0) && (tmp3 == SD_OK) && (timeout > 0))
    {
        tmp1 = uSdHandle.DmaTransferCplt;
        tmp2 = uSdHandle.SdTransferCplt;
        tmp3 = (SD_Error_t)uSdHandle.SdTransferErr;
        timeout--;
    }

    timeout = Timeout;

    // Wait until the Tx transfer is no longer active
    while(((SDMMC1->STA & SDMMC_STA_TXACT) != 0) && (timeout > 0))
    {
        timeout--;
    }

    // Send stop command in multiblock write
    if(uSdHandle.SdOperation == SD_MULTIPLE_BLOCK)
    {
        errorstate = SD_StopTransfer();
    }

    if((timeout == 0) && (errorstate == SD_OK))
    {
        errorstate = SD_DATA_TIMEOUT;
    }

    // Clear all the static flags
    SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;

    // Return error state
    if(uSdHandle.SdTransferErr != SD_OK)
    {
        return (SD_Error_t)(uSdHandle.SdTransferErr);
    }

    // Wait until write is complete
    while(SD_GetStatus() != SD_TRANSFER_OK){};

    return errorstate;
}

/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  startaddr: Start byte address
  * @param  endaddr: End byte address
  * @retval SD Card error state
  */
SD_Error_t SD_Erase(uint64_t startaddr, uint64_t endaddr)
{
    SD_Error_t errorstate = SD_OK;

    uint32_t delay         = 0;
    __IO uint32_t maxdelay = 0;
    uint8_t cardstate      = 0;

    // Check if the card command class supports erase command
    if(((uSdHandle.CSD[1] >> 20) & SD_CCCC_ERASE) == 0)
    {
        return SD_REQUEST_NOT_APPLICABLE;
    }

    // Get max delay value
    maxdelay = 120000 / (((SDMMC1->CLKCR) & 0xFF) + 2);

    if((SDMMC1->RESP1 & SD_CARD_LOCKED) == SD_CARD_LOCKED)
    {
        return SD_LOCK_UNLOCK_FAILED;
    }

    // Get start and end block for high capacity cards
    if(uSdHandle.CardType == HIGH_CAPACITY_SD_CARD)
    {
        startaddr /= 512;
        endaddr   /= 512;
    }

    // According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33)
    if ((uSdHandle.CardType == STD_CAPACITY_SD_CARD_V1_1) || (uSdHandle.CardType == STD_CAPACITY_SD_CARD_V2_0) ||
        (uSdHandle.CardType == HIGH_CAPACITY_SD_CARD))
    {
        // Send CMD32 SD_ERASE_GRP_START with argument as addr
        SDMMC1->ARG  =  (uint32_t)startaddr;
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_SD_ERASE_GRP_START | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        if((errorstate = SD_CmdResp1Error( SD_CMD_SD_ERASE_GRP_START)) != SD_OK)
        {
            return errorstate;
        }

        // Send CMD33 SD_ERASE_GRP_END with argument as addr
        SDMMC1->ARG  =  (uint32_t)endaddr;
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_SD_ERASE_GRP_END | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        if((errorstate = SD_CmdResp1Error( SD_CMD_SD_ERASE_GRP_END)) != SD_OK)
        {
            return errorstate;
        }
    }

    // Send CMD38 ERASE
    SDMMC1->ARG  =  (uint32_t)0;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_ERASE  | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((errorstate = SD_CmdResp1Error( SD_CMD_ERASE)) != SD_OK)
    {
        return errorstate;
    }

    for(; delay < maxdelay; delay++);

    // Wait until the card is in programming state
    errorstate = SD_IsCardProgramming( &cardstate);

    delay = SD_DATATIMEOUT;

    while((delay > 0) && (errorstate == SD_OK) && ((cardstate == SD_CARD_PROGRAMMING) || (cardstate == SD_CARD_RECEIVING)))
    {
        errorstate = SD_IsCardProgramming( &cardstate);
        delay--;
    }

    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Returns information about specific card.
  *         contains all SD cardinformation
  * @retval SD Card error state
  */
SD_Error_t SD_GetCardInfo(void)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t tmp = 0;

    SD_CardInfo.CardType = (uint8_t)(uSdHandle.CardType);
    SD_CardInfo.RCA      = (uint16_t)(uSdHandle.RCA);

    // Byte 0
    tmp = (uSdHandle.CSD[0] & 0xFF000000) >> 24;
    SD_CardInfo.SD_csd.CSDStruct      = (uint8_t)((tmp & 0xC0) >> 6);
    SD_CardInfo.SD_csd.SysSpecVersion = (uint8_t)((tmp & 0x3C) >> 2);
    SD_CardInfo.SD_csd.Reserved1      = tmp & 0x03;

    // Byte 1
    tmp = (uSdHandle.CSD[0] & 0x00FF0000) >> 16;
    SD_CardInfo.SD_csd.TAAC = (uint8_t)tmp;

    // Byte 2
    tmp = (uSdHandle.CSD[0] & 0x0000FF00) >> 8;
    SD_CardInfo.SD_csd.NSAC = (uint8_t)tmp;

    // Byte 3
    tmp = uSdHandle.CSD[0] & 0x000000FF;
    SD_CardInfo.SD_csd.MaxBusClkFrec = (uint8_t)tmp;

    // Byte 4
    tmp = (uSdHandle.CSD[1] & 0xFF000000) >> 24;
    SD_CardInfo.SD_csd.CardComdClasses = (uint16_t)(tmp << 4);

    // Byte 5
    tmp = (uSdHandle.CSD[1] & 0x00FF0000) >> 16;
    SD_CardInfo.SD_csd.CardComdClasses |= (uint16_t)((tmp & 0xF0) >> 4);
    SD_CardInfo.SD_csd.RdBlockLen       = (uint8_t)(tmp & 0x0F);

    // Byte 6
    tmp = (uSdHandle.CSD[1] & 0x0000FF00) >> 8;
    SD_CardInfo.SD_csd.PartBlockRead   = (uint8_t)((tmp & 0x80) >> 7);
    SD_CardInfo.SD_csd.WrBlockMisalign = (uint8_t)((tmp & 0x40) >> 6);
    SD_CardInfo.SD_csd.RdBlockMisalign = (uint8_t)((tmp & 0x20) >> 5);
    SD_CardInfo.SD_csd.DSRImpl         = (uint8_t)((tmp & 0x10) >> 4);
    SD_CardInfo.SD_csd.Reserved2       = 0; /*!< Reserved */

    if((uSdHandle.CardType == STD_CAPACITY_SD_CARD_V1_1) || (uSdHandle.CardType == STD_CAPACITY_SD_CARD_V2_0))
    {
        SD_CardInfo.SD_csd.DeviceSize = (tmp & 0x03) << 10;

        // Byte 7
        tmp = (uint8_t)(uSdHandle.CSD[1] & 0x000000FF);
        SD_CardInfo.SD_csd.DeviceSize |= (tmp) << 2;

        // Byte 8
        tmp = (uint8_t)((uSdHandle.CSD[2] & 0xFF000000) >> 24);
        SD_CardInfo.SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

        SD_CardInfo.SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
        SD_CardInfo.SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

        // Byte 9
        tmp = (uint8_t)((uSdHandle.CSD[2] & 0x00FF0000) >> 16);
        SD_CardInfo.SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
        SD_CardInfo.SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
        SD_CardInfo.SD_csd.DeviceSizeMul      = (tmp & 0x03) << 1;

        // Byte 10
        tmp = (uint8_t)((uSdHandle.CSD[2] & 0x0000FF00) >> 8);
        SD_CardInfo.SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;

        SD_CardInfo.CardCapacity  = (SD_CardInfo.SD_csd.DeviceSize + 1) ;
        SD_CardInfo.CardCapacity *= (1 << (SD_CardInfo.SD_csd.DeviceSizeMul + 2));
        SD_CardInfo.CardBlockSize = 1 << (SD_CardInfo.SD_csd.RdBlockLen);
        SD_CardInfo.CardCapacity *= SD_CardInfo.CardBlockSize;
    }
    else if(uSdHandle.CardType == HIGH_CAPACITY_SD_CARD)
    {
        // Byte 7
        tmp = (uint8_t)(uSdHandle.CSD[1] & 0x000000FF);
        SD_CardInfo.SD_csd.DeviceSize = (tmp & 0x3F) << 16;

        // Byte 8
        tmp = (uint8_t)((uSdHandle.CSD[2] & 0xFF000000) >> 24);

        SD_CardInfo.SD_csd.DeviceSize |= (tmp << 8);

        // Byte 9
        tmp = (uint8_t)((uSdHandle.CSD[2] & 0x00FF0000) >> 16);

        SD_CardInfo.SD_csd.DeviceSize |= (tmp);

        // Byte 10
        tmp = (uint8_t)((uSdHandle.CSD[2] & 0x0000FF00) >> 8);

        SD_CardInfo.CardCapacity  = ((SD_CardInfo.SD_csd.DeviceSize + 1)) * 512 * 1024;
        SD_CardInfo.CardBlockSize = 512;
    }
    else
    {
        // Not supported card type
        errorstate = SD_ERROR;
    }

    SD_CardInfo.SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
    SD_CardInfo.SD_csd.EraseGrMul  = (tmp & 0x3F) << 1;

    // Byte 11
    tmp = (uint8_t)(uSdHandle.CSD[2] & 0x000000FF);
    SD_CardInfo.SD_csd.EraseGrMul     |= (tmp & 0x80) >> 7;
    SD_CardInfo.SD_csd.WrProtectGrSize = (tmp & 0x7F);

    // Byte 12
    tmp = (uint8_t)((uSdHandle.CSD[3] & 0xFF000000) >> 24);
    SD_CardInfo.SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
    SD_CardInfo.SD_csd.ManDeflECC        = (tmp & 0x60) >> 5;
    SD_CardInfo.SD_csd.WrSpeedFact       = (tmp & 0x1C) >> 2;
    SD_CardInfo.SD_csd.MaxWrBlockLen     = (tmp & 0x03) << 2;

    // Byte 13
    tmp = (uint8_t)((uSdHandle.CSD[3] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_csd.MaxWrBlockLen      |= (tmp & 0xC0) >> 6;
    SD_CardInfo.SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
    SD_CardInfo.SD_csd.Reserved3           = 0;
    SD_CardInfo.SD_csd.ContentProtectAppli = (tmp & 0x01);

    // Byte 14
    tmp = (uint8_t)((uSdHandle.CSD[3] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
    SD_CardInfo.SD_csd.CopyFlag         = (tmp & 0x40) >> 6;
    SD_CardInfo.SD_csd.PermWrProtect    = (tmp & 0x20) >> 5;
    SD_CardInfo.SD_csd.TempWrProtect    = (tmp & 0x10) >> 4;
    SD_CardInfo.SD_csd.FileFormat       = (tmp & 0x0C) >> 2;
    SD_CardInfo.SD_csd.ECC              = (tmp & 0x03);

    // Byte 15
    tmp = (uint8_t)(uSdHandle.CSD[3] & 0x000000FF);
    SD_CardInfo.SD_csd.CSD_CRC   = (tmp & 0xFE) >> 1;
    SD_CardInfo.SD_csd.Reserved4 = 1;

    // Byte 0
    tmp = (uint8_t)((uSdHandle.CID[0] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ManufacturerID = tmp;

    // Byte 1
    tmp = (uint8_t)((uSdHandle.CID[0] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.OEM_AppliID = tmp << 8;

    // Byte 2
    tmp = (uint8_t)((uSdHandle.CID[0] & 0x000000FF00) >> 8);
    SD_CardInfo.SD_cid.OEM_AppliID |= tmp;

    // Byte 3
    tmp = (uint8_t)(uSdHandle.CID[0] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdName1 = tmp << 24;

    // Byte 4
    tmp = (uint8_t)((uSdHandle.CID[1] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdName1 |= tmp << 16;

    // Byte 5
    tmp = (uint8_t)((uSdHandle.CID[1] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.ProdName1 |= tmp << 8;

    // Byte 6
    tmp = (uint8_t)((uSdHandle.CID[1] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ProdName1 |= tmp;

    // Byte 7
    tmp = (uint8_t)(uSdHandle.CID[1] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdName2 = tmp;

    // Byte 8
    tmp = (uint8_t)((uSdHandle.CID[2] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdRev = tmp;

    // Byte 9
    tmp = (uint8_t)((uSdHandle.CID[2] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.ProdSN = tmp << 24;

    // Byte 10
    tmp = (uint8_t)((uSdHandle.CID[2] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ProdSN |= tmp << 16;

    // Byte 11
    tmp = (uint8_t)(uSdHandle.CID[2] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdSN |= tmp << 8;

    // Byte 12
    tmp = (uint8_t)((uSdHandle.CID[3] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdSN |= tmp;

    // Byte 13
    tmp = (uint8_t)((uSdHandle.CID[3] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.Reserved1   |= (tmp & 0xF0) >> 4;
    SD_CardInfo.SD_cid.ManufactDate = (tmp & 0x0F) << 8;

    // Byte 14
    tmp = (uint8_t)((uSdHandle.CID[3] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ManufactDate |= tmp;

    // Byte 15
    tmp = (uint8_t)(uSdHandle.CID[3] & 0x000000FF);
    SD_CardInfo.SD_cid.CID_CRC   = (tmp & 0xFE) >> 1;
    SD_CardInfo.SD_cid.Reserved2 = 1;

    return errorstate;
}

/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Enables wide bus operation for the requested card if supported by
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode
  *          This parameter can be one of the following values:
  *            @arg SDMMC_BUS_WIDE_8B: 8-bit data transfer (Only for MMC)
  *            @arg SDMMC_BUS_WIDE_4B: 4-bit data transfer
  *            @arg SDMMC_BUS_WIDE_1B: 1-bit data transfer
  * @retval SD Card error state
  */
SD_Error_t SD_WideBusOperationConfig(uint32_t WideMode)
{
    SD_Error_t errorstate;

    // MMC Card does not support this feature
    if(uSdHandle.CardType == MULTIMEDIA_CARD)
    {
        errorstate = SD_UNSUPPORTED_FEATURE;
    }
    else if((uSdHandle.CardType == STD_CAPACITY_SD_CARD_V1_1) || (uSdHandle.CardType == STD_CAPACITY_SD_CARD_V2_0) ||\
            (uSdHandle.CardType == HIGH_CAPACITY_SD_CARD))
    {
             if(WideMode == SDMMC_BUS_WIDE_8B)  errorstate = SD_UNSUPPORTED_FEATURE;
        else if(WideMode == SDMMC_BUS_WIDE_4B)  errorstate = SD_WideBusEnable();
        else if(WideMode == SDMMC_BUS_WIDE_1B)  errorstate = SD_WideBusDisable();
        else                                    errorstate = SD_INVALID_PARAMETER;  // WideMode is not a valid argument

        if(errorstate == SD_OK)
        {
            // Configure the SDMMC peripheral
            SDMMC1->CLKCR &= ~(uint32_t)CLKCR_CLEAR_MASK;
            SDMMC1->CLKCR |=  (uint32_t)WideMode;
        }
    }

    return errorstate;
}

/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Aborts an ongoing data transfer.
  * @retval SD Card error state
  */
SD_Error_t SD_StopTransfer(void)
{
    // Send CMD12 STOP_TRANSMISSION
    SDMMC1->ARG  =  (uint32_t)0;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_STOP_TRANSMISSION | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    return SD_CmdResp1Error(SD_CMD_STOP_TRANSMISSION);;
}

/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Switches the SD card to High Speed mode.
  *         This API must be used after "Transfer State"
  * @retval SD Card error state
  */
SD_Error_t HAL_SD_HighSpeed(void)
{
    SD_Error_t errorstate = SD_OK;

    uint8_t SD_hs[64]  = {0};
    uint32_t SD_scr[2] = {0, 0};
    uint32_t SD_SPEC   = 0;
    uint32_t count = 0, *tempbuff = (uint32_t *)SD_hs;

    // Initialize the Data control register
    SDMMC1->DCTRL = 0;

    // Get SCR Register
    if((errorstate = SD_FindSCR( SD_scr)) != SD_OK)
    {
        return errorstate;
    }

    // Test the Version supported by the card
    SD_SPEC = (SD_scr[1]  & 0x01000000) | (SD_scr[1]  & 0x02000000);

    if(SD_SPEC != SD_ALLZERO)
    {
        // Set Block Size for Card
        SDMMC1->ARG  =  (uint32_t)64;
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_SET_BLOCKLEN | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        if((errorstate = SD_CmdResp1Error( SD_CMD_SET_BLOCKLEN)) != SD_OK)
        {
            return errorstate;
        }

        // Configure the SD DPSM (Data Path State Machine)
        SDMMC1->DTIMER = SD_DATATIMEOUT;
        SDMMC1->DLEN   = 64;
        SDMMC1->DCTRL &= ~(uint32_t)DCTRL_CLEAR_MASK;
        SDMMC1->DCTRL |=  (uint32_t)(SDMMC_DATABLOCK_SIZE_64B | SDMMC_DCTRL_DTDIR | SDMMC_DCTRL_DTEN);

        // Send CMD6 switch mode
        SDMMC1->ARG  =  (uint32_t)0x80FFFF01;
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_HS_SWITCH | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        if((errorstate = SD_CmdResp1Error( SD_CMD_HS_SWITCH)) != SD_OK)
        {
            return errorstate;
        }

        while((SDMMC1->STA & (SDMMC_STA_RXOVERR | SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_DBCKEND)) == 0)
        {
            if((SDMMC1->STA & SDMMC_STA_RXFIFOHF) != 0)
            {
                for (count = 0; count < 8; count++)
                {
                    *(tempbuff + count) = SDMMC1->FIFO;
                }

                tempbuff += 8;
            }
        }

        if((SDMMC1->STA & SDMMC_STA_DTIMEOUT) != 0)
        {
            SDMMC1->ICR = SDMMC_ICR_DTIMEOUTC;
            return SD_DATA_TIMEOUT;
        }
        else if((SDMMC1->STA & SDMMC_STA_DCRCFAIL) != 0)
        {
            SDMMC1->ICR = SDMMC_ICR_DCRCFAILC;
            return SD_DATA_CRC_FAIL;
        }
        else if((SDMMC1->STA & SDMMC_STA_RXOVERR) != 0)
        {
            SDMMC1->ICR = SDMMC_ICR_RXOVERRC;
            return SD_RX_OVERRUN;
        }

        count = SD_DATATIMEOUT;

        while(((SDMMC1->STA & SDMMC_STA_RXDAVL) != 0) && (count > 0))
        {
            *tempbuff = SDMMC1->FIFO;
            tempbuff++;
            count--;
        }

        // Clear all the static flags
        SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;

        // Test if the switch mode HS is ok
        if((SD_hs[13]& 2) != 2)
        {
            errorstate = SD_UNSUPPORTED_FEATURE;
        }
    }

    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Returns the current SD card's status.
  * @param  pSDstatus: Pointer to the buffer that will contain the SD card status
  * @retval SD Card error state
  */
SD_Error_t HAL_SD_SendSDStatus(uint32_t *pSDstatus)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t count = 0;

    // Check SD response
    if((SDMMC1->RESP1 & SD_CARD_LOCKED) == SD_CARD_LOCKED)
    {
        return SD_LOCK_UNLOCK_FAILED;
    }

    // Set block size for card if it is not equal to current block size for card
    SDMMC1->ARG  =  (uint32_t)64;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_SET_BLOCKLEN | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((errorstate = SD_CmdResp1Error( SD_CMD_SET_BLOCKLEN)) != SD_OK)
    {
        return errorstate;
    }

    // Send CMD55
    SDMMC1->ARG  =  (uint32_t)(uSdHandle.RCA << 16);
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_APP_CMD | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((errorstate = SD_CmdResp1Error( SD_CMD_APP_CMD)) != SD_OK)
    {
        return errorstate;
    }

    // Configure the SD DPSM (Data Path State Machine)
    SDMMC1->DTIMER = SD_DATATIMEOUT;
    SDMMC1->DLEN   = 64;
    SDMMC1->DCTRL &= ~(uint32_t)DCTRL_CLEAR_MASK;
    SDMMC1->DCTRL |=  (uint32_t)(SDMMC_DATABLOCK_SIZE_64B | SDMMC_DCTRL_DTDIR | SDMMC_DCTRL_DTEN);

    // Send ACMD13 (SD_APP_STAUS)  with argument as card's RCA */
    SDMMC1->ARG  =  (uint32_t)0;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_SD_APP_STATUS | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((errorstate = SD_CmdResp1Error( SD_CMD_SD_APP_STATUS)) != SD_OK)
    {
        return errorstate;
    }

    // Get status data
    while((SDMMC1->STA & (SDMMC_STA_RXOVERR | SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_DBCKEND)) == 0)
    {
        if((SDMMC1->STA & SDMMC_STA_RXFIFOHF) != 0)
        {
            for(count = 0; count < 8; count++)
            {
                *(pSDstatus + count) = SDMMC1->FIFO;
            }

            pSDstatus += 8;
        }
    }

    if((SDMMC1->STA & SDMMC_STA_DTIMEOUT) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_DTIMEOUTC;
        return SD_DATA_TIMEOUT;
    }
    else if((SDMMC1->STA & SDMMC_STA_DCRCFAIL) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_DCRCFAILC;
        return SD_DATA_CRC_FAIL;
    }
    else if((SDMMC1->STA & SDMMC_STA_RXOVERR) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_RXOVERRC;
        return SD_RX_OVERRUN;
    }

    count = SD_DATATIMEOUT;
    while(((SDMMC1->STA & SDMMC_STA_RXDAVL) != 0) && (count > 0))
    {
        *pSDstatus = SDMMC1->FIFO;
        pSDstatus++;
        count--;
    }

    // Clear all the static status flags
    SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;
    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Gets the current sd card data status.
  * @retval Data Transfer state
  */
SD_TransferStateTypedef SD_GetStatus(void)
{
    uint32_t            resp1 = 0;
    SD_CardStateTypedef CardState = SD_CARD_TRANSFER;

  // Get SD card state
    if(SD_SendStatus(&resp1) != SD_OK) CardState =  SD_CARD_ERROR;
    else                               CardState = (SD_CardStateTypedef)((resp1 >> 9) & 0x0F);

    // Find SD status according to card state
    if     (CardState == SD_CARD_TRANSFER)  return SD_TRANSFER_OK;
    else if(CardState == SD_CARD_ERROR)     return SD_TRANSFER_ERROR;
    else                                    return SD_TRANSFER_BUSY;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Gets the SD card status.
  * @retval SD Card error state
  */
SD_Error_t SD_GetCardStatus(SD_CardStatusTypedef* pCardStatus)
{
    SD_Error_t errorstate;
    uint32_t tmp = 0;
    uint32_t sd_status[16];

    if((errorstate = SD_SendSDStatus( sd_status)) != SD_OK)
    {
        return errorstate;
    }

    // Byte 0
    tmp = (sd_status[0] & 0xC0) >> 6;
    pCardStatus->DAT_BUS_WIDTH = (uint8_t)tmp;

    // Byte 0
    tmp = (sd_status[0] & 0x20) >> 5;
    pCardStatus->SECURED_MODE = (uint8_t)tmp;

    // Byte 2
    tmp = (sd_status[2] & 0xFF);
    pCardStatus->SD_CARD_TYPE = (uint8_t)(tmp << 8);

    // Byte 3
    tmp = (sd_status[3] & 0xFF);
    pCardStatus->SD_CARD_TYPE |= (uint8_t)tmp;

    // Byte 4
    tmp = (sd_status[4] & 0xFF);
    pCardStatus->SIZE_OF_PROTECTED_AREA = (uint8_t)(tmp << 24);

    // Byte 5
    tmp = (sd_status[5] & 0xFF);
    pCardStatus->SIZE_OF_PROTECTED_AREA |= (uint8_t)(tmp << 16);

    // Byte 6
    tmp = (sd_status[6] & 0xFF);
    pCardStatus->SIZE_OF_PROTECTED_AREA |= (uint8_t)(tmp << 8);

    // Byte 7
    tmp = (sd_status[7] & 0xFF);
    pCardStatus->SIZE_OF_PROTECTED_AREA |= (uint8_t)tmp;

    // Byte 8
    tmp = (sd_status[8] & 0xFF);
    pCardStatus->SPEED_CLASS = (uint8_t)tmp;

    // Byte 9
    tmp = (sd_status[9] & 0xFF);
    pCardStatus->PERFORMANCE_MOVE = (uint8_t)tmp;

    // Byte 10
    tmp = (sd_status[10] & 0xF0) >> 4;
    pCardStatus->AU_SIZE = (uint8_t)tmp;

    // Byte 11
    tmp = (sd_status[11] & 0xFF);
    pCardStatus->ERASE_SIZE = (uint8_t)(tmp << 8);

    // Byte 12
    tmp = (sd_status[12] & 0xFF);
    pCardStatus->ERASE_SIZE |= (uint8_t)tmp;

    // Byte 13
    tmp = (sd_status[13] & 0xFC) >> 2;
    pCardStatus->ERASE_TIMEOUT = (uint8_t)tmp;

    // Byte 13
    tmp = (sd_status[13] & 0x3);
    pCardStatus->ERASE_OFFSET = (uint8_t)tmp;

    return SD_OK;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Selects od Deselects the corresponding card.
  * @param  addr: Address of the card to be selected
  * @retval SD Card error state
  */
static SD_Error_t SD_Select_Deselect(uint64_t addr)
{
    // Send CMD7 SDMMC_SEL_DESEL_CARD
    SDMMC1->ARG  =  (uint32_t)addr;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_SEL_DESEL_CARD | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // check for error conditions
    return SD_CmdResp1Error( SD_CMD_SEL_DESEL_CARD);
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Enquires cards about their operating voltage and configures clock
  *         controls and stores SD information that will be needed in future
  *         in the SD handle.
  * @retval SD Card error state
  */
static SD_Error_t SD_PowerON(void)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t response = 0, count = 0, validvoltage = 0;
    uint32_t sdtype = SD_STD_CAPACITY;

    // Power ON Sequence -------------------------------------------------------
    SDMMC1->CLKCR &= ~SDMMC_CLKCR_CLKEN;        // Disable SDMMC Clock
    SDMMC1->POWER = SDMMC_POWER_PWRCTRL;        // Set Power State to ON

    // 1ms: required power up waiting time before starting the SD initialization sequence
    uint32_t tickstart = 0;
    tickstart = Tick;
    while((Tick - tickstart) < 1) {};

    SDMMC1->CLKCR |= SDMMC_CLKCR_CLKEN;           // Enable SDMMC Clock

    // CMD0: GO_IDLE_STATE -----------------------------------------------------
    // No CMD response required
    SDMMC1->ARG  =  0;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_GO_IDLE_STATE | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((errorstate = SD_CmdError()) != SD_OK)
    {
        // CMD Response Timeout (wait for CMDSENT flag)
        return errorstate;
    }

    // CMD8: SEND_IF_COND ------------------------------------------------------
    // Send CMD8 to verify SD card interface operating condition
    // Argument: - [31:12]: Reserved (shall be set to '0')
    //- [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    //- [7:0]: Check Pattern (recommended 0xAA)
    // CMD Response: R7 */
    SDMMC1->ARG  =  SD_CHECK_PATTERN;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_SDMMC_SEND_IF_COND | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((errorstate = SD_CmdResp7Error()) == SD_OK)
    {
        // SD Card 2.0
        uSdHandle.CardType = STD_CAPACITY_SD_CARD_V2_0;
        sdtype        = SD_HIGH_CAPACITY;
    }

    // Send CMD55
    SDMMC1->ARG  =  (uint32_t)0;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_APP_CMD | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    // If errorstate is Command Timeout, it is a MMC card
    // If errorstate is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch) or SD card 1.x
    if((errorstate = SD_CmdResp1Error(SD_CMD_APP_CMD)) == SD_OK)
    {
        // SD CARD
        // Send ACMD41 SD_APP_OP_COND with Argument 0x80100000
        while((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
        {

            // SEND CMD55 APP_CMD with RCA as 0
            SDMMC1->ARG  =  (uint32_t)0;
            SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
            SDMMC1->CMD |=  (uint32_t)(SD_CMD_APP_CMD | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

            // Check for error conditions
            if((errorstate = SD_CmdResp1Error( SD_CMD_APP_CMD)) != SD_OK)
            {
                return errorstate;
            }

            // Send CMD41
            SDMMC1->ARG  =  SD_VOLTAGE_WINDOW_SD | sdtype;
            SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
            SDMMC1->CMD |=  (uint32_t)(SD_CMD_SD_APP_OP_COND | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

            // Check for error conditions
            if((errorstate = SD_CmdResp3Error()) != SD_OK)
            {
                return errorstate;
            }

            response = SDMMC1->RESP1;                               // Get command response
            validvoltage = (((response >> 31) == 1) ? 1 : 0);       // Get operating voltage
            count++;
        }

        if(count >= SD_MAX_VOLT_TRIAL)
        {
            return SD_INVALID_VOLTRANGE;
        }

        if((response & SD_HIGH_CAPACITY) == SD_HIGH_CAPACITY) /* (response &= SD_HIGH_CAPACITY) */
        {
            uSdHandle.CardType = HIGH_CAPACITY_SD_CARD;
        }
    } // else MMC Card

    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Turns the SDMMC output signals off.
  * @retval SD Card error state
  */
static void SD_PowerOFF(void)
{
   // Set Power State to OFF
   SDMMC1->POWER = (uint32_t)0;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Returns the current card's status.
  * @param  pCardStatus: pointer to the buffer that will contain the SD card
  *         status (Card Status register)
  * @retval SD Card error state
  */
static SD_Error_t SD_SendStatus(uint32_t *pCardStatus)
{
    SD_Error_t errorstate;

    // Send Status command
    SDMMC1->ARG  =  (uint32_t)(uSdHandle.RCA << 16);
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_SEND_STATUS | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    if((errorstate = SD_CmdResp1Error( SD_CMD_SEND_STATUS)) == SD_OK)
    {
        *pCardStatus = SDMMC1->RESP1;
    }

    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks for error conditions for CMD0.
  * @retval SD Card error state
  */
static SD_Error_t SD_CmdError(void)
{
    uint32_t timeout, tmp;

    timeout = SDMMC_CMD0TIMEOUT;

    tmp = SDMMC1->STA & SDMMC_STA_CMDSENT;

    while((timeout > 0) && (tmp == 0))
    {
        tmp = SDMMC1->STA & SDMMC_STA_CMDSENT;
        timeout--;
    }

    if(timeout == 0)
    {
        return SD_CMD_RSP_TIMEOUT;
    }

    // Clear all the static flags
    SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;
    return SD_OK;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks for error conditions for R7 response.
  * @retval SD Card error state
  */
static SD_Error_t SD_CmdResp7Error(void)
{
    uint32_t timeout = SDMMC_CMD0TIMEOUT, tmp;

    tmp = SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT);

    while((tmp == 0) && (timeout > 0))
    {
        tmp = SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT);
        timeout--;
    }

    tmp = SDMMC1->STA & SDMMC_STA_CTIMEOUT;

    if((timeout == 0) || (tmp != 0))
    {
        // Card is not V2.0 compliant or card does not support the set voltage range
        SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;
        return SD_CMD_RSP_TIMEOUT;
    }

    if((SDMMC1->STA & SDMMC_STA_CMDREND) != 0)
    {
        // Card is SD V2.0 compliant
        SDMMC1->ICR = SDMMC_ICR_CMDRENDC;
        return SD_OK;
    }

    return SD_ERROR;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks for error conditions for R1 response.
  * @param  SD_CMD: The sent command index
  * @retval SD Card error state
  */
static SD_Error_t SD_CmdResp1Error(uint8_t SD_CMD)
{
    uint32_t response_r1;

    while((SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) == 0){};

    if((SDMMC1->STA & SDMMC_STA_CTIMEOUT) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;
        return SD_CMD_RSP_TIMEOUT;
    }
    else if((SDMMC1->STA & SDMMC_STA_CCRCFAIL) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_CCRCFAILC;
        return SD_CMD_CRC_FAIL;
    }

    // Check response received is of desired command
    if((uint8_t)SDMMC1->RESPCMD != SD_CMD)
    {
        return SD_ILLEGAL_CMD;
    }

    // Clear all the static flags
    SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;

    // We have received response, retrieve it for analysis
    response_r1 = SDMMC1->RESP1;

    if((response_r1 & SD_OCR_ERRORBITS)             == SD_ALLZERO)                      return SD_OK;
    if((response_r1 & SD_OCR_ADDR_OUT_OF_RANGE)     == SD_OCR_ADDR_OUT_OF_RANGE)        return SD_ADDR_OUT_OF_RANGE;
    if((response_r1 & SD_OCR_ADDR_MISALIGNED)       == SD_OCR_ADDR_MISALIGNED)          return SD_ADDR_MISALIGNED;
    if((response_r1 & SD_OCR_BLOCK_LEN_ERR)         == SD_OCR_BLOCK_LEN_ERR)            return SD_BLOCK_LEN_ERR;
    if((response_r1 & SD_OCR_ERASE_SEQ_ERR)         == SD_OCR_ERASE_SEQ_ERR)            return SD_ERASE_SEQ_ERR;
    if((response_r1 & SD_OCR_BAD_ERASE_PARAM)       == SD_OCR_BAD_ERASE_PARAM)          return SD_BAD_ERASE_PARAM;
    if((response_r1 & SD_OCR_WRITE_PROT_VIOLATION)  == SD_OCR_WRITE_PROT_VIOLATION)     return SD_WRITE_PROT_VIOLATION;
    if((response_r1 & SD_OCR_LOCK_UNLOCK_FAILED)    == SD_OCR_LOCK_UNLOCK_FAILED)       return SD_LOCK_UNLOCK_FAILED;
    if((response_r1 & SD_OCR_COM_CRC_FAILED)        == SD_OCR_COM_CRC_FAILED)           return SD_COM_CRC_FAILED;
    if((response_r1 & SD_OCR_ILLEGAL_CMD)           == SD_OCR_ILLEGAL_CMD)              return SD_ILLEGAL_CMD;
    if((response_r1 & SD_OCR_CARD_ECC_FAILED)       == SD_OCR_CARD_ECC_FAILED)          return SD_CARD_ECC_FAILED;
    if((response_r1 & SD_OCR_CC_ERROR)              == SD_OCR_CC_ERROR)                 return SD_CC_ERROR;
    if((response_r1 & SD_OCR_GENERAL_UNKNOWN_ERROR) == SD_OCR_GENERAL_UNKNOWN_ERROR)    return SD_GENERAL_UNKNOWN_ERROR;
    if((response_r1 & SD_OCR_STREAM_READ_UNDERRUN)  == SD_OCR_STREAM_READ_UNDERRUN)     return SD_STREAM_READ_UNDERRUN;
    if((response_r1 & SD_OCR_STREAM_WRITE_OVERRUN)  == SD_OCR_STREAM_WRITE_OVERRUN)     return SD_STREAM_WRITE_OVERRUN;
    if((response_r1 & SD_OCR_CID_CSD_OVERWRITE)     == SD_OCR_CID_CSD_OVERWRITE)        return SD_CID_CSD_OVERWRITE;
    if((response_r1 & SD_OCR_WP_ERASE_SKIP)         == SD_OCR_WP_ERASE_SKIP)            return SD_WP_ERASE_SKIP;
    if((response_r1 & SD_OCR_CARD_ECC_DISABLED)     == SD_OCR_CARD_ECC_DISABLED)        return SD_CARD_ECC_DISABLED;
    if((response_r1 & SD_OCR_ERASE_RESET)           == SD_OCR_ERASE_RESET)              return SD_ERASE_RESET;
    if((response_r1 & SD_OCR_AKE_SEQ_ERROR)         == SD_OCR_AKE_SEQ_ERROR)            return SD_AKE_SEQ_ERROR;
    return SD_OK;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks for error conditions for R3 (OCR) response.
  * @retval SD Card error state
  */
static SD_Error_t SD_CmdResp3Error(void)
{
    SD_Error_t errorstate = SD_OK;

    while((SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) == 0){};

    if((SDMMC1->STA & SDMMC_STA_CTIMEOUT) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;
        return SD_CMD_RSP_TIMEOUT;
    }

    // Clear all the static flags
    SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;
    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks for error conditions for R2 (CID or CSD) response.
  * @retval SD Card error state
  */
static SD_Error_t SD_CmdResp2Error(void)
{
    SD_Error_t errorstate = SD_OK;

    while((SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) == 0){};

    if((SDMMC1->STA & SDMMC_STA_CTIMEOUT) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;
        return SD_CMD_RSP_TIMEOUT;
    }
    else if((SDMMC1->STA & SDMMC_STA_CCRCFAIL) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_CCRCFAILC;
        return SD_CMD_CRC_FAIL;
    }
    // Clear all the static flags
    SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;
    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks for error conditions for R6 (RCA) response.
  * @param  SD_CMD: The sent command index
  * @param  pRCA: Pointer to the variable that will contain the SD card relative
  *         address RCA
  * @retval SD Card error state
  */
static SD_Error_t SD_CmdResp6Error(uint8_t SD_CMD, uint16_t *pRCA)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t response_r1;

    while((SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) == 0){};

    if((SDMMC1->STA & SDMMC_STA_CTIMEOUT) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;
        return SD_CMD_RSP_TIMEOUT;
    }
    else if((SDMMC1->STA & SDMMC_STA_CCRCFAIL) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_CCRCFAILC;
        return SD_CMD_CRC_FAIL;
    }

    // Check response received is of desired command
    if((uint8_t)SDMMC1->RESPCMD != SD_CMD)
    {
        return SD_ILLEGAL_CMD;
    }

    SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;       // Clear all the static flags
    response_r1 = SDMMC1->RESP1;                // We have received response, retrieve it.
    if((response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)) == SD_ALLZERO)
    {
        *pRCA = (uint16_t) (response_r1 >> 16);
        return errorstate;
    }

    if((response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR) == SD_R6_GENERAL_UNKNOWN_ERROR)  return SD_GENERAL_UNKNOWN_ERROR;
    if((response_r1 & SD_R6_ILLEGAL_CMD) == SD_R6_ILLEGAL_CMD)                      return SD_ILLEGAL_CMD;
    if((response_r1 & SD_R6_COM_CRC_FAILED) == SD_R6_COM_CRC_FAILED)                return SD_COM_CRC_FAILED;
    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Enables the SDMMC wide bus mode.
  * @retval SD Card error state
  */
static SD_Error_t SD_WideBusEnable(void)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t scr[2] = {0, 0};

    if((SDMMC1->RESP1 & SD_CARD_LOCKED) == SD_CARD_LOCKED)
    {
        return SD_LOCK_UNLOCK_FAILED;
    }

    // Get SCR Register
    if((errorstate = SD_FindSCR(scr)) != SD_OK)
    {
        return errorstate;
    }

    // If requested card supports wide bus operation
    if((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)
    {
        // Send CMD55 APP_CMD with argument as card's RCA.
        SDMMC1->ARG = (uint32_t)(uSdHandle.RCA << 16);
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_APP_CMD | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        if((errorstate = SD_CmdResp1Error(SD_CMD_APP_CMD)) != SD_OK)
        {
            return errorstate;
        }

        // Send ACMD6 APP_CMD with argument as 2 for wide bus mode
        SDMMC1->ARG  =  (uint32_t)2;
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_APP_SD_SET_BUSWIDTH | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        errorstate = SD_CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);
        return errorstate;
    }
    else
    {
        errorstate = SD_REQUEST_NOT_APPLICABLE;
        return errorstate;
    }
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Disables the SDMMC wide bus mode.
  * @retval SD Card error state
  */
static SD_Error_t SD_WideBusDisable(void)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t scr[2] = {0, 0};

    if((SDMMC1->RESP1 & SD_CARD_LOCKED) == SD_CARD_LOCKED)
    {
        return SD_LOCK_UNLOCK_FAILED;
    }

    // Get SCR Register
    if((errorstate = SD_FindSCR(scr)) != SD_OK)
    {
        return errorstate;
    }

    // If requested card supports 1 bit mode operation
    if((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
    {
        // Send CMD55 APP_CMD with argument as card's RCA
        SDMMC1->ARG = (uint32_t)(uSdHandle.RCA << 16);
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_APP_CMD | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        if((errorstate = SD_CmdResp1Error( SD_CMD_APP_CMD)) != SD_OK)
        {
            return errorstate;
        }

        // Send ACMD6 APP_CMD with argument as 0 for single bus mode
        SDMMC1->ARG  =  (uint32_t)0;                // Set the SDMMC Argument value
        SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;   // Set SDMMC command parameters
        SDMMC1->CMD |=  (uint32_t)(SD_CMD_APP_SD_SET_BUSWIDTH | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

        // Check for error conditions
        return SD_CmdResp1Error( SD_CMD_APP_SD_SET_BUSWIDTH);;
    }
    else
    {
        return SD_REQUEST_NOT_APPLICABLE;
    }
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Finds the SD card SCR register value.
  * @param  pSCR: pointer to the buffer that will contain the SCR value
  * @retval SD Card error state
  */
static SD_Error_t SD_FindSCR(uint32_t *pSCR)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t index = 0;
    uint32_t tempscr[2] = {0, 0};

    // Set Block Size To 8 Bytes
    // Send CMD55 APP_CMD with argument as card's RCA
    SDMMC1->ARG  =  (uint32_t)8;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_SET_BLOCKLEN | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);
    // Check for error conditions
    errorstate = SD_CmdResp1Error( SD_CMD_SET_BLOCKLEN);

    if(errorstate != SD_OK)
    {
        return errorstate;
    }

    // Send CMD55 APP_CMD with argument as card's RCA
    SDMMC1->ARG  =  (uint32_t)((uSdHandle.RCA) << 16);
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_APP_CMD | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    errorstate = SD_CmdResp1Error( SD_CMD_APP_CMD);

    if(errorstate != SD_OK)
    {
        return errorstate;
    }

    SDMMC1->DTIMER = SD_DATATIMEOUT;
    SDMMC1->DLEN   = 8;
    SDMMC1->DCTRL &= ~(uint32_t)DCTRL_CLEAR_MASK;
    SDMMC1->DCTRL |=  (uint32_t)(SDMMC_DATABLOCK_SIZE_8B | SDMMC_DCTRL_DTDIR | SDMMC_DCTRL_DTEN);

    // Send ACMD51 SD_APP_SEND_SCR with argument as 0
    SDMMC1->ARG  =  (uint32_t)0;
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_SD_APP_SEND_SCR | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    // Check for error conditions
    errorstate = SD_CmdResp1Error( SD_CMD_SD_APP_SEND_SCR);

    if(errorstate != SD_OK)
    {
        return errorstate;
    }

    while((SDMMC1->STA & (SDMMC_STA_RXOVERR | SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_DBCKEND)) == 0)
    {
        if((SDMMC1->STA & SDMMC_STA_RXDAVL) != 0)
        {
            *(tempscr + index) = SDMMC1->FIFO;
            index++;
        }
    }

    if((SDMMC1->STA & SDMMC_STA_DTIMEOUT) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_DTIMEOUTC;
        errorstate = SD_DATA_TIMEOUT;
    }
    else if((SDMMC1->STA & SDMMC_STA_DCRCFAIL) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_DCRCFAILC;
        errorstate = SD_DATA_CRC_FAIL;
    }
    else if((SDMMC1->STA & SDMMC_STA_RXOVERR) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_RXOVERRC;
        errorstate = SD_RX_OVERRUN;
    }
    else
    {
        // Clear all the static flags
        SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;

        *(pSCR + 1) = ((tempscr[0] & SD_0TO7BITS) << 24)  | ((tempscr[0] & SD_8TO15BITS) << 8) |
                      ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

        *(pSCR) = ((tempscr[1] & SD_0TO7BITS) << 24)  | ((tempscr[1] & SD_8TO15BITS) << 8) |
                  ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);
    }

    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks if the SD card is in programming state.
  * @param  pStatus: pointer to the variable that will contain the SD card state
  * @retval SD Card error state
  */
static SD_Error_t SD_IsCardProgramming(uint8_t *pStatus)
{
    SD_Error_t errorstate = SD_OK;
    uint32_t responseR1 = 0;

    SDMMC1->ARG  =  (uint32_t)(uSdHandle.RCA << 16);
    SDMMC1->CMD &= ~(uint32_t)CMD_CLEAR_MASK;
    SDMMC1->CMD |=  (uint32_t)(SD_CMD_SEND_STATUS | SDMMC_CMD_RESPONSE_SHORT | SDMMC_CMD_CPSMEN);

    while((SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) == 0){};

    if((SDMMC1->STA & SDMMC_STA_CTIMEOUT) != 0)
    {
        errorstate = SD_CMD_RSP_TIMEOUT;
        SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;
        return errorstate;
    }
    else if((SDMMC1->STA & SDMMC_STA_CCRCFAIL) != 0)
    {
        errorstate = SD_CMD_CRC_FAIL;
        SDMMC1->ICR = SDMMC_ICR_CCRCFAILC;
        return errorstate;
    }
    else
    {
        // No error flag set
    }

    // Check response received is of desired command
    if((uint32_t)SDMMC1->RESPCMD != SD_CMD_SEND_STATUS)
    {
        errorstate = SD_ILLEGAL_CMD;
        return errorstate;
    }

    // Clear all the static flags
    SDMMC1->ICR = SDMMC_ICR_STATIC_FLAGS;

    // We have received response, retrieve it for analysis
    responseR1 = SDMMC1->RESP1;

    // Find out card status
    *pStatus = (uint8_t)((responseR1 >> 9) & 0x0000000F);

    if((responseR1 & SD_OCR_ERRORBITS)              == SD_ALLZERO)                      return errorstate;
    if((responseR1 & SD_OCR_ADDR_OUT_OF_RANGE)      == SD_OCR_ADDR_OUT_OF_RANGE)        return SD_ADDR_OUT_OF_RANGE;
    if((responseR1 & SD_OCR_ADDR_MISALIGNED)        == SD_OCR_ADDR_MISALIGNED)          return SD_ADDR_MISALIGNED;
    if((responseR1 & SD_OCR_BLOCK_LEN_ERR)          == SD_OCR_BLOCK_LEN_ERR)            return SD_BLOCK_LEN_ERR;
    if((responseR1 & SD_OCR_ERASE_SEQ_ERR)          == SD_OCR_ERASE_SEQ_ERR)            return SD_ERASE_SEQ_ERR;
    if((responseR1 & SD_OCR_BAD_ERASE_PARAM)        == SD_OCR_BAD_ERASE_PARAM)          return SD_BAD_ERASE_PARAM;
    if((responseR1 & SD_OCR_WRITE_PROT_VIOLATION)   == SD_OCR_WRITE_PROT_VIOLATION)     return SD_WRITE_PROT_VIOLATION;
    if((responseR1 & SD_OCR_LOCK_UNLOCK_FAILED)     == SD_OCR_LOCK_UNLOCK_FAILED)       return SD_LOCK_UNLOCK_FAILED;
    if((responseR1 & SD_OCR_COM_CRC_FAILED)         == SD_OCR_COM_CRC_FAILED)           return SD_COM_CRC_FAILED;
    if((responseR1 & SD_OCR_ILLEGAL_CMD)            == SD_OCR_ILLEGAL_CMD)              return SD_ILLEGAL_CMD;
    if((responseR1 & SD_OCR_CARD_ECC_FAILED)        == SD_OCR_CARD_ECC_FAILED)          return SD_CARD_ECC_FAILED;
    if((responseR1 & SD_OCR_CC_ERROR)               == SD_OCR_CC_ERROR)                 return SD_CC_ERROR;
    if((responseR1 & SD_OCR_GENERAL_UNKNOWN_ERROR)  == SD_OCR_GENERAL_UNKNOWN_ERROR)    return SD_GENERAL_UNKNOWN_ERROR;
    if((responseR1 & SD_OCR_STREAM_READ_UNDERRUN)   == SD_OCR_STREAM_READ_UNDERRUN)     return SD_STREAM_READ_UNDERRUN;
    if((responseR1 & SD_OCR_STREAM_WRITE_OVERRUN)   == SD_OCR_STREAM_WRITE_OVERRUN)     return SD_STREAM_WRITE_OVERRUN;
    if((responseR1 & SD_OCR_CID_CSD_OVERWRITE)      == SD_OCR_CID_CSD_OVERWRITE)        return SD_CID_CSD_OVERWRITE;
    if((responseR1 & SD_OCR_WP_ERASE_SKIP)          == SD_OCR_WP_ERASE_SKIP)            return SD_WP_ERASE_SKIP;
    if((responseR1 & SD_OCR_CARD_ECC_DISABLED)      == SD_OCR_CARD_ECC_DISABLED)        return SD_CARD_ECC_DISABLED;
    if((responseR1 & SD_OCR_ERASE_RESET)            == SD_OCR_ERASE_RESET)              return SD_ERASE_RESET;
    if((responseR1 & SD_OCR_AKE_SEQ_ERROR)          == SD_OCR_AKE_SEQ_ERROR)            return SD_AKE_SEQ_ERROR;
    return errorstate;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Initialize the SDMMC1 module, DMA, and IO
  */
void SD_Initialize(void)
{
    uint32_t prioritygroup;

     // Reset SDIO Module
    RCC->APB2RSTR |=  RCC_APB2RSTR_SDMMC1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SDMMC1RST;

    // Enable SDIO clock
    RCC->APB2ENR |= RCC_APB2ENR_SDMMC1EN;

    // Enable DMA2 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	IO_PinInit(GPIOC,               RCC_AHB1ENR_GPIOCEN, GPIO_PIN_8,    GPIO_MODER_ALTERNATE, GPIO_TYPE_PIN_PP,      GPIO_OSPEEDR_VERY_HIGH, 12);
	IO_PinInit(GPIOC,               RCC_AHB1ENR_GPIOCEN, GPIO_PIN_9,    GPIO_MODER_ALTERNATE, GPIO_TYPE_PIN_PP,      GPIO_OSPEEDR_VERY_HIGH, 12);
	IO_PinInit(GPIOC,               RCC_AHB1ENR_GPIOCEN, GPIO_PIN_10,   GPIO_MODER_ALTERNATE, GPIO_TYPE_PIN_PP,      GPIO_OSPEEDR_VERY_HIGH, 12);
	IO_PinInit(GPIOC,               RCC_AHB1ENR_GPIOCEN, GPIO_PIN_11,   GPIO_MODER_ALTERNATE, GPIO_TYPE_PIN_PP,      GPIO_OSPEEDR_VERY_HIGH, 12);
	IO_PinInit(GPIOC,               RCC_AHB1ENR_GPIOCEN, GPIO_PIN_12,   GPIO_MODER_ALTERNATE, GPIO_TYPE_PIN_PP,      GPIO_OSPEEDR_VERY_HIGH, 12);
	IO_PinInit(GPIOD,               RCC_AHB1ENR_GPIODEN, GPIO_PIN_2,    GPIO_MODER_ALTERNATE, GPIO_TYPE_PIN_PP,      GPIO_OSPEEDR_VERY_HIGH, 12);
	IO_PinInit(SD_DETECT_GPIO_PORT, RCC_AHB1ENR_GPIOCEN, SD_DETECT_PIN, GPIO_MODER_INPUT,     GPIO_TYPE_PIN_PULL_UP, GPIO_OSPEEDR_LOW,       0);

    prioritygroup = NVIC_GetPriorityGrouping();

    // NVIC configuration for SDIO interrupts
    NVIC_SetPriority(SDMMC1_IRQn, NVIC_EncodePriority(prioritygroup, 5, 0));
    NVIC_EnableIRQ(SDMMC1_IRQn);

    // Initialize DMA2 channel 3 for RX from SD CARD
    DMA2_Stream3->CR   = 0;                                                 // Reset DMA Stream control register
    DMA2_Stream3->PAR  = (uint32_t)&SDMMC1->FIFO;
    DMA2->LIFCR        = IFCR_CLEAR_MASK_STREAM3;                           // Clear all interrupt flags
    DMA2_Stream3->CR   = (DMA_CHANNEL_4        | DMA_SxCR_PFCTRL        |   // Prepare the DMA Stream configuration
                          DMA_MINC_ENABLE      | DMA_PDATAALIGN_WORD    |   // And write to DMA Stream CR register
                          DMA_MDATAALIGN_WORD  | DMA_PRIORITY_VERY_HIGH |
                          DMA_MBURST_INC4      | DMA_PBURST_INC4);

    DMA2_Stream3->FCR  = (DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);                 // Configuration FIFO control register

    // Initialize DMA2 channel 6 for TX to SD CARD
    DMA2_Stream6->CR   = 0;                                                 // Reset DMA Stream control register
    DMA2_Stream6->PAR  = (uint32_t)&SDMMC1->FIFO;
    DMA2->HIFCR        = IFCR_CLEAR_MASK_STREAM6;                           // Clear all interrupt flags
    DMA2_Stream6->CR   = (DMA_CHANNEL_4        | DMA_SxCR_PFCTRL        |   // Prepare the DMA Stream configuration
                          DMA_MINC_ENABLE      | DMA_PDATAALIGN_WORD    |   // And write to DMA Stream CR register
                          DMA_MDATAALIGN_WORD  | DMA_PRIORITY_VERY_HIGH |
                          DMA_MBURST_INC4      | DMA_PBURST_INC4        |
                          DMA_MEMORY_TO_PERIPH);
    DMA2_Stream6->FCR  = (DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);                 // Configuration FIFO control register

    // NVIC configuration for DMA transfer complete interrupt
    NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(prioritygroup, 6, 0));
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    NVIC_SetPriority(DMA2_Stream6_IRQn, NVIC_EncodePriority(prioritygroup, 6, 0));
    NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}


/** -----------------------------------------------------------------------------------------------------------------*/
DSTATUS FATFS_SD_disk_initialize(void)
{
	uint8_t SD_state = MSD_OK;

	// Check if SD card is present
	if(SD_IsDetected() != SD_PRESENT)   return MSD_ERROR_SD_NOT_PRESENT;
	// HAL SD initialization
    if(HAL_SD_Init() != SD_OK)          SD_state = MSD_ERROR;

	// Configure SD Bus width
	if(SD_state == MSD_OK)
	{
		// Enable wide operation
		if(SD_WideBusOperationConfig(SDMMC_BUS_WIDE_4B) != SD_OK) SD_state = MSD_ERROR;
		else SD_state = MSD_OK;
	}

	// Configure the SDCARD device
	Stat = STA_NOINIT;
	if (SD_state == MSD_OK) Stat &= ~STA_NOINIT;

	return Stat;
}


/** -----------------------------------------------------------------------------------------------------------------*/
DSTATUS FATFS_SD_disk_status(void)
{
	Stat = STA_NOINIT;

	// Check SDCARD status
	if (SD_GetStatus() == MSD_OK) Stat &= ~STA_NOINIT;
	else Stat |= STA_NOINIT;

	return Stat;
}


/** -----------------------------------------------------------------------------------------------------------------*/
DRESULT FATFS_SD_disk_ioctl(BYTE cmd, void *buff)
{
	DRESULT res = RES_ERROR;

	// Check if init OK
	if(Stat & STA_NOINIT) return RES_NOTRDY;

	switch(cmd)
	{
		// Make sure that no pending write process
		case CTRL_SYNC:
        {
			res = RES_OK;
			break;
        }

		// Get number of sectors on the disk (DWORD)
		case GET_SECTOR_COUNT:
        {
			SD_GetCardInfo();
			*(DWORD *)buff = SD_CardInfo.CardCapacity / BLOCK_SIZE;
			res = RES_OK;
			break;
        }

		// Get R/W sector size (WORD)
		case GET_SECTOR_SIZE:
        {
			*(WORD *)buff = BLOCK_SIZE;
			res = RES_OK;
			break;
        }

		// Get erase block size in unit of sector (DWORD)
		case GET_BLOCK_SIZE:
        {
			*(DWORD*)buff = BLOCK_SIZE;
			break;
        }

		default:
        {
			res = RES_PARERR;
        }
	}

	return res;
}


/** -----------------------------------------------------------------------------------------------------------------*/
DRESULT FATFS_SD_disk_read(BYTE *buff, DWORD sector, UINT count)
{
	uint8_t SD_state = MSD_OK;

	// Read block(s) in DMA transfer mode
	if(SD_ReadBlocks_DMA((uint32_t *)buff, (uint64_t) (sector * BLOCK_SIZE), BLOCK_SIZE, count) != SD_OK)
    {
        SD_state = MSD_ERROR;
    }

	// Wait until transfer is complete
	if(SD_state == MSD_OK)
	{
		if(SD_CheckReadOperation((uint32_t)SD_DATATIMEOUT) != SD_OK) SD_state = MSD_ERROR;
		else SD_state = MSD_OK;
	}

	if(SD_state == MSD_OK)
    {
        return RES_OK;
    }

	return RES_ERROR;
}


/** -----------------------------------------------------------------------------------------------------------------*/
DRESULT FATFS_SD_disk_write(const BYTE *buff, DWORD sector, UINT count)
{
	uint8_t SD_state = MSD_OK;

	// Write block(s) in DMA transfer mode
	if(SD_WriteBlocks_DMA((uint32_t *)buff, (sector * BLOCK_SIZE), BLOCK_SIZE, count) != SD_OK)
    {
        SD_state = MSD_ERROR;
    }

	// Wait until transfer is complete
	if(SD_state == MSD_OK)
	{
		if(SD_CheckWriteOperation((uint32_t)SD_DATATIMEOUT) != SD_OK) SD_state = MSD_ERROR;
		else SD_state = MSD_OK;
	}

	if(SD_state == MSD_OK)
    {
        return RES_OK;
    }

	return RES_ERROR;
}


/** -----------------------------------------------------------------------------------------------------------------*/
FRESULT FATFS_GetDriveSize(char* pStr, FATFS_Size_t* SizeStruct)
{
	FATFS*  pFS;
    DWORD   FreeCluster;
	FRESULT Result;

    // Get volume information and free clusters of drive
    if((Result = f_getfree(pStr, &FreeCluster, &pFS)) == FR_OK)
    {
        // Get total sectors and free sectors
        SizeStruct->Total = (pFS->n_fatent - 2) * pFS->csize * 0.5;
        SizeStruct->Free = FreeCluster * pFS->csize * 0.5;
    }

    return Result;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  This function handles SD card interrupt request.
  */
void SDMMC1_IRQHandler(void)
{
    // Check for SDMMC interrupt flags
    if((SDMMC1->STA & SDMMC_STA_DATAEND) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_DATAENDC;
        uSdHandle.SdTransferCplt = 1;           // SD transfer is complete
        uSdHandle.SdTransferErr  = SD_OK;       // No transfer error
    }
    else if((SDMMC1->STA & SDMMC_STA_DCRCFAIL) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_DCRCFAILC;
        uSdHandle.SdTransferErr = SD_DATA_CRC_FAIL;
    }
    else if((SDMMC1->STA & SDMMC_STA_DTIMEOUT) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_DTIMEOUTC;
        uSdHandle.SdTransferErr = SD_DATA_TIMEOUT;
    }
    else if((SDMMC1->STA & SDMMC_STA_RXOVERR) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_RXOVERRC;
        uSdHandle.SdTransferErr = SD_RX_OVERRUN;
    }
    else if((SDMMC1->STA & SDMMC_STA_TXUNDERR) != 0)
    {
        SDMMC1->ICR = SDMMC_ICR_TXUNDERRC;
        uSdHandle.SdTransferErr = SD_TX_UNDERRUN;
    }

    // Disable all SDMMC peripheral interrupt sources
    SDMMC1->MASK &= ~(SDMMC_MASK_DCRCFAILIE | SDMMC_MASK_DTIMEOUTIE | SDMMC_MASK_DATAENDIE  |
                      SDMMC_MASK_TXFIFOHEIE | SDMMC_MASK_RXFIFOHFIE | SDMMC_MASK_TXUNDERRIE |
                      SDMMC_MASK_RXOVERRIE);
}

/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  This function handles DMA2 Stream 3 interrupt request.
  */
void DMA2_Stream3_IRQHandler(void)
{
    // Transfer Error Interrupt management
    if((DMA2->LISR & DMA_LISR_TEIF3) != 0)
    {
        if((DMA2_Stream3->CR & DMA_SxCR_TEIE) != 0)
        {
            DMA2_Stream3->CR   &= ~DMA_SxCR_TEIE;       // Disable the transfer error interrupt
            DMA2->LIFCR = DMA_LIFCR_CTEIF3;             // Clear the transfer error flag
        }
    }

    // FIFO Error Interrupt management
    if((DMA2->LISR & DMA_LISR_FEIF3) != 0)
    {
        if((DMA2_Stream3->FCR & DMA_SxFCR_FEIE) != 0)
        {
            DMA2_Stream3->FCR   &= ~DMA_SxFCR_FEIE;     // Disable the FIFO Error interrupt
            DMA2->LIFCR = DMA_LIFCR_CFEIF3;             // Clear the FIFO error flag
        }
    }

    // Direct Mode Error Interrupt management
    if((DMA2->LISR & DMA_LISR_DMEIF3) != 0)
    {
        if((DMA2_Stream3->CR & DMA_SxCR_DMEIE) != 0)
        {
            DMA2_Stream3->CR   &= ~DMA_SxCR_DMEIE;       // Disable the direct mode Error interrupt
            DMA2->LIFCR = DMA_LIFCR_CDMEIF3;             // Clear the FIFO error flag
        }
    }

    // Half Transfer Complete Interrupt management
    if((DMA2->LISR & DMA_LISR_HTIF3) != 0)
    {
        if((DMA2_Stream3->CR & DMA_SxCR_HTIE) != 0)
        {
            if(((DMA2_Stream3->CR) & (uint32_t)(DMA_SxCR_DBM)) != 0)    // Multi_Buffering mode enabled
            {
                DMA2->LIFCR = DMA_LIFCR_CHTIF3;                         // Clear the half transfer complete flag
            }
            else
            {
                if((DMA2_Stream3->CR & DMA_SxCR_CIRC) == 0)             // Disable the half transfer interrupt if the DMA mode is not CIRCULAR
                {
                    DMA2_Stream3->CR   &= ~DMA_SxCR_HTIE;               // Disable the half transfer interrupt
                }

                DMA2->LIFCR = DMA_LIFCR_CHTIF3;                         // Clear the half transfer complete flag
            }
        }
    }

    // Transfer Complete Interrupt management
    if((DMA2->LISR & DMA_LISR_TCIF3) != 0)
    {
        if((DMA2_Stream3->CR & DMA_SxCR_TCIE) != 0)
        {
            if((DMA2_Stream3->CR & (uint32_t)(DMA_SxCR_DBM)) != 0)
            {
                DMA2->LIFCR = DMA_LIFCR_CTCIF3;                         // Clear the transfer complete flag
            }
            else //Disable the transfer complete interrupt if the DMA mode is not CIRCULAR
            {
                if((DMA2_Stream3->CR & DMA_SxCR_CIRC) == 0)
                {
                    DMA2_Stream3->CR &= ~DMA_SxCR_TCIE;                 // Disable the transfer complete interrupt
                }

                DMA2->LIFCR = DMA_LIFCR_CTCIF3;                         // Clear the transfer complete flag
                SD_DMA_RxCplt();
            }
        }
    }
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  This function handles DMA2 Stream 6 interrupt request.
  */
void DMA2_Stream6_IRQHandler(void)
{
    // Transfer Error Interrupt management
    if((DMA2->HISR & DMA_HISR_TEIF6) != 0)
    {
        if((DMA2_Stream6->CR & DMA_SxCR_TEIE) != 0)
        {
            DMA2_Stream6->CR   &= ~DMA_SxCR_TEIE;       // Disable the transfer error interrupt
            DMA2->HIFCR = DMA_HIFCR_CTEIF6;             // Clear the transfer error flag
        }
    }

    // FIFO Error Interrupt management
    if((DMA2->HISR & DMA_HISR_FEIF6) != 0)
    {
        if((DMA2_Stream6->FCR & DMA_SxFCR_FEIE) != 0)
        {
            DMA2_Stream6->FCR   &= ~DMA_SxFCR_FEIE;     // Disable the FIFO Error interrupt
            DMA2->HIFCR = DMA_HIFCR_CFEIF6;             // Clear the FIFO error flag
        }
    }

    // Direct Mode Error Interrupt management
    if((DMA2->HISR & DMA_HISR_DMEIF6) != 0)
    {
        if((DMA2_Stream6->CR & DMA_SxCR_DMEIE) != 0)
        {
            DMA2_Stream6->CR   &= ~DMA_SxCR_DMEIE;       // Disable the direct mode Error interrupt
            DMA2->HIFCR = DMA_HIFCR_CDMEIF6;             // Clear the FIFO error flag
        }
    }

    // Half Transfer Complete Interrupt management
    if((DMA2->HISR & DMA_HISR_HTIF6) != 0)
    {
        if((DMA2_Stream6->CR & DMA_SxCR_HTIE) != 0)
        {
            if(((DMA2_Stream6->CR) & (uint32_t)(DMA_SxCR_DBM)) != 0)    // Multi_Buffering mode enabled
            {
                DMA2->HIFCR = DMA_HIFCR_CHTIF6;                         // Clear the half transfer complete flag
            }
            else
            {
                if((DMA2_Stream6->CR & DMA_SxCR_CIRC) == 0)             // Disable the half transfer interrupt if the DMA mode is not CIRCULAR
                {
                    DMA2_Stream6->CR &= ~DMA_SxCR_HTIE;                 // Disable the half transfer interrupt
                }

                DMA2->HIFCR = DMA_HIFCR_CHTIF6;                         // Clear the half transfer complete flag
            }
        }
    }

    // Transfer Complete Interrupt management
    if((DMA2->HISR & DMA_HISR_TCIF6) != 0)
    {
        if((DMA2_Stream6->CR & DMA_SxCR_TCIE) != 0)
        {
            if((DMA2_Stream6->CR & (uint32_t)(DMA_SxCR_DBM)) != 0)
            {
                DMA2->HIFCR = DMA_HIFCR_CTCIF6;                         // Clear the transfer complete flag
            }
            else //Disable the transfer complete interrupt if the DMA mode is not CIRCULAR
            {
                if((DMA2_Stream6->CR & DMA_SxCR_CIRC) == 0)
                {
                    DMA2_Stream6->CR   &= ~DMA_SxCR_TCIE;               // Disable the transfer complete interrupt
                }

                DMA2->HIFCR = DMA_HIFCR_CTCIF6;                         // Clear the transfer complete flag
                SD_DMA_TxCplt();
            }
        }
    }
}


/* ------------------------------------------------------------------------------------------------------------------*/