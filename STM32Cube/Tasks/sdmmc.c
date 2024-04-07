#include "sdmmc.h"


void sdmmcTask(void *argument) {
	// mount that fucker
	FATFS fs;

	FRESULT mount_res = f_mount(&fs, "", 1);

	if (mount_res != FR_OK) {
		goto UNMOUNT;
	}

	// check if the fuckin file system is actually fucking working
	FILINFO info;
	const char *file_path = "testing.txt";

	FRESULT fs_stat_res = f_stat(file_path, &info);

	if (fs_stat_res != FR_OK) {
		goto UNMOUNT;
	}

	// create some fuckin file
//    FIL file;
//    FRESULT open_file_res = f_open(&file, "a.txt", FA_WRITE | FA_CREATE_ALWAYS);
//    if (open_file_res != FR_OK) {
//        goto UNMOUNT;
//    }

	UNMOUNT:
	f_mount(NULL, "", 0);


	for (;;) {
		/* empty */
	}

	vTaskDelete( NULL );


}

