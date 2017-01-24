/* Includes ---------------------------------------------------------------------------------------------------------*/

#include "diskio.h"		        // FatFs lower layer API
#include "fatfs_sd_sdio.h"	    // Header file of existing MMC/SDC contorl module
#include "bsp.h"

/** -----------------------------------------------------------------------------------------------------------------*/
/**		Disk Status
  *
  * @brief  Disk Status
  * @param  BYTE pdrv		Physical drive number to identify the drive
  * @retval DSTATUS
  *
  */
DSTATUS disk_status(BYTE pdrv)
{
	VAR_UNUSED(pdrv);
    return FATFS_SD_disk_status();
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**		Initialize a Drive
  *
  * @brief  Initialize a Drive
  * @param  BYTE pdrv		Physical drive number to identify the drive
  * @retval DSTATUS
  *
  */
DSTATUS disk_initialize(BYTE pdrv)
{
	VAR_UNUSED(pdrv);
    return FATFS_SD_disk_initialize();
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**		Read Sector(s)
  *
  * @brief  Read Sector(s)
  * @param  BYTE pdrv		Physical drive number to identify the drive
  * @param  BYTE *buff      Data buffer to store read data
  * @param  DWORD sector    Sector address in LBA
  * @param  UINT count		Number of sectors to read
  * @retval DRESULT
  *
  */
DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
	VAR_UNUSED(pdrv);
    return FATFS_SD_disk_read(buff, sector, count);
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**		Write Sector(s)
  *
  * @brief  Write Sector(s)
  * @param  BYTE pdrv		    Physical drive number to identify the drive
  * @param  const BYTE *buff    Data buffer to store read data
  * @param  DWORD sector        Sector address in LBA
  * @param  UINT count		    Number of sectors to write
  * @retval DRESULT
  *
  */
#if _USE_WRITE
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
	VAR_UNUSED(pdrv);
    return FATFS_SD_disk_write(buff, sector, count);
}
#endif


/** -----------------------------------------------------------------------------------------------------------------*/
/**		Miscellaneous Functions
  *
  * @brief  Miscellaneous Functions
  * @param  BYTE pdrv		Physical drive number to identify the drive
  * @param  BYTE cmd        Control code
  * @param  void *buff      Buffer to send/receive control data
  * @retval DRESULT
  *
  */
#if _USE_IOCTL
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
	VAR_UNUSED(pdrv);
    return FATFS_SD_disk_ioctl(cmd, buff);
}
#endif
