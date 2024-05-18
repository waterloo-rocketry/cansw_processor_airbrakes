#include "sdmmc.h"

UART_HandleTypeDef* sdmmcHuart;

void print(const char *formatter, ...) {
	va_list args;
	va_start(args, formatter);

	char output[1024] = {0};

	vsnprintf(output, 1024, formatter, args);
	va_end(args);

	HAL_UART_Transmit(sdmmcHuart, (uint8_t *)output, strlen(output), 10);
}

FRESULT list_dir (char *path)
{

	print("READING DIR '/%s'\n", path);
    FRESULT res;
    DIR dir;
    FILINFO fno;
    int nfile, ndir;


    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        nfile = ndir = 0;
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Error or end of dir */
            if (fno.fattrib & AM_DIR) {       /* Directory */
                print("   <DIR>   %s\n", fno.fname);
                ndir++;
            } else {
                print("%10u %s\n", fno.fsize, fno.fname);
                nfile++;
            }
        }
        f_closedir(&dir);
        print("%d dirs, %d files.\n", ndir, nfile);
    } else {
        print("Failed to open \"%s\". (%u)\n", path, res);
    }
    return res;
}


void sdmmcTask(void *argument) {
	sdmmcHuart = (UART_HandleTypeDef*)argument;

	print("\n----- STARTING SDMMC TESTING -----\n");

	FATFS fs;

	print("Mounting SDCard\n");
	FRESULT mount_res = f_mount(&fs, "", 0);

	if (mount_res != FR_OK) {
		print("Failed to mount: %d\n", mount_res);
		goto UNMOUNT;
	}

	print("\n");

    FIL file;
    print("Creating file\n");
    FRESULT open_file_res = f_open(&file, "b.txt", FA_OPEN_EXISTING | FA_WRITE);
    if (open_file_res != FR_OK) {
    	print("Failed to open: %d\n", open_file_res);
        goto UNMOUNT;
    } else {
    	print("File created!\n");
//    	int writtenAmount = f_printf(&file, "Testing writing into a file\0");
    	char toWrite[1024] = "main contnet 123123123123\ncontent on new line @gmail.com\nagain with Gumption\n";
    	print("Attempting to write the following to the file:");
    	print("\n----------------------------\n");
    	print(toWrite);
    	print("\n----------------------------\n");

    	int output = 0;
    	int result = f_write(&file, toWrite, sizeof(toWrite), &output);

    	print("Write responded with: %d\n", result);
    	f_close(&file);
    }

    print("\n");

    print("Opening file for reading\n");
    open_file_res = f_open(&file, "b.txt", FA_READ);
    if (open_file_res != FR_OK) {
    	print("Failed to open file: %d\n", open_file_res);
    	goto UNMOUNT;
    } else {
    	print("File opened!\n");

    	char line[1024];
    	int readBytes = 0;

    	print("Reading a file:");
    	print("\n----------------------------\n");
        while (f_gets(line, sizeof line, &file)) {
        	print("%10s", line);
        }
        print("\n----------------------------\n");

    }

    print("\n");

    list_dir("");



	UNMOUNT:
	print("----- STOPPING SDMMC TESTING -----\n");
	f_mount(0, "", 0);


	for (;;) {
		/* empty */
	}

	vTaskDelete( NULL );


}

