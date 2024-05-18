#include "sdmmc.h"

UART_HandleTypeDef* sdmmcHuart;


void printLine(const char* message) {
	HAL_UART_Transmit(sdmmcHuart, (uint8_t *)message, strlen(message), 10);
	HAL_UART_Transmit(sdmmcHuart, "\n", 1, 10);
}


void sdmmcTask(void *argument) {
	sdmmcHuart = (UART_HandleTypeDef*)argument;

	printLine("testing things");

	// mount that fucker
	FATFS fs; // Causes: FR_NOT_READY f_mount (pullup?, how enable?)
//	FATFS *fs = NULL; // Causes: FR_OK f_mount, FR_NOT_ENABLED f_open

	printLine("mounting");
	FRESULT mount_res = f_mount(&fs, "", 0);

	if (mount_res != FR_OK) {
		printLine("FAiled to mount");
		goto UNMOUNT;
	}

	// check if the fuckin file system is actually fucking working
//	FILINFO info;
//	const char *file_path = "testing.txt";
//
//	FRESULT fs_stat_res = f_stat(file_path, &info);
//
//	if (fs_stat_res != FR_OK) {
//		goto UNMOUNT;
//	}

//	 create some fuckin file
    FIL file;
    printLine("Opening file");
    FRESULT open_file_res = f_open(&file, "a.txt", FA_CREATE_ALWAYS);
    if (open_file_res != FR_OK) {
    	printLine("Failed to open");
        goto UNMOUNT;
    } else {
    	printLine("Open worked!");
    	f_close(&file);
    }

	UNMOUNT:
	printLine("Unmounting");
	f_mount(0, "", 0);


	for (;;) {
		/* empty */
	}

	vTaskDelete( NULL );


}

