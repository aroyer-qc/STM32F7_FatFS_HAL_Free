/* Includes ---------------------------------------------------------------------------------------------------------*/

#include "bsp.h"
#include "ff.h"
#include "diskio.h"
#include "fatfs_sd_sdio.h"
#include <string.h>

/* Variables --------------------------------------------------------------------------------------------------------*/

FATFS           FS;
FIL             File;
FATFS_Size_t    CardSize;

/** -----------------------------------------------------------------------------------------------------------------*/
int main (void)
{
	Initialize();

	if(f_mount(&FS, "SD:", 1) == FR_OK)
	{
		f_mkdir("SD:/FatFs_TEST");

        // Try to open file
        if(f_open(&File, "SD:/FatFs_TEST/test.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
        {
            FATFS_GetDriveSize("SD:", &CardSize);                               // Read SDCARD size
            f_printf(&File, "Total card size: %u kBytes\n", CardSize.Total);    // Format string and write total card size to file
            f_printf(&File, "Free card size:  %u kBytes\n", CardSize.Free);     // Format string for free card size and write free card size to file
            f_close(&File); 				                                    // Close file
            ledOn();                                                            // Turn led ON
        }

        f_mount(NULL, "SD:", 1);                                                // Unmount SDCARD
	}

	while(1);
}

/* ------------------------------------------------------------------------------------------------------------------*/
