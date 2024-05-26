#include "sdmmc.h"

extern UART_HandleTypeDef huart4;

const char *logsPath = "/LOGS";
char logFileName[LOG_FILE_NAME_SIZE];

/*static void print(const char *formatter, ...) {*/
/*	va_list args;*/
/*	va_start(args, formatter);*/
/**/
/*	char output[1024] = {0};*/
/**/
/*	vsnprintf(output, 1024, formatter, args);*/
/*	va_end(args);*/
/**/
/*	HAL_UART_Transmit(&huart4, (uint8_t *)output, strlen(output), 10);*/
/*}*/

/*static FRESULT list_dir(char *path) {*/
/*	print("READING DIR '%s'\n", path);*/
/*	FRESULT res;*/
/*	DIR dir;*/
/*	FILINFO fno;*/
/*	int nfile, ndir;*/
/**/
/*	res = f_opendir(&dir, path); /* Open the directory */*/
/*	if (res == FR_OK) {*/
/*		nfile = ndir = 0;*/
/*		for (;;) {*/
/*			res = f_readdir(&dir, &fno); /* Read a directory item */*/
/*			if (res != FR_OK || fno.fname[0] == 0)*/
/*				break;                    /* Error or end of dir */*/
/*			if (fno.fattrib & AM_DIR) { /* Directory */*/
/*				print("   <DIR>   %s\n", fno.fname);*/
/*				ndir++;*/
/*			} else {*/
/*				print("%10u %s\n", fno.fsize, fno.fname);*/
/*				nfile++;*/
/*			}*/
/*		}*/
/*		f_closedir(&dir);*/
/*		print("%d dirs, %d files.\n", ndir, nfile);*/
/*	} else {*/
/*		print("Failed to open \"%s\". (%u)\n", path, res);*/
/*	}*/
/**/
/*	return res;*/
/*}*/

static int computeFolderSize(const char *path) {
	DIR dir;
	FILINFO fno;
	int nfile = 0;

	if (f_opendir(&dir, path) == FR_OK) {
		while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
			if (!(fno.fattrib & AM_DIR))
				++nfile;
		}
		f_closedir(&dir);
	}

	return nfile;
}

void initUniqueLogFileName() {
	int nextValidFileNumber = computeFolderSize(logsPath) - 1;
	while (snprintf(logFileName, LOG_FILE_NAME_SIZE, "%s/%d.txt", logsPath,
				 ++nextValidFileNumber) > 0 &&
		f_stat(logFileName, NULL) == FR_OK)
		;
}

/*static void sdmmcTestingTask() {*/
/*	/*huart4 = (UART_HandleTypeDef *)argument;*/*/
/**/
/*	print("\n----- STARTING SDMMC TESTING -----\n");*/
/**/
/*	FATFS fs;*/
/**/
/*	print("Mounting SDCard\n");*/
/*	FRESULT mount_res = f_mount(&fs, "", 0);*/
/**/
/*	if (mount_res != FR_OK) {*/
/*		print("Failed to mount SD card: %d\n", mount_res);*/
/*		goto UNMOUNT;*/
/*	}*/
/**/
/*	print("\n");*/
/**/
/*	// ensure log folder exists*/
/*	int res = f_mkdir(logsPath);*/
/**/
/*	// init unique log file name*/
/*	initUniqueLogFileName(logsPath);*/
/*	snprintf(logFileName, LOG_FILE_NAME_SIZE, "%s/%d.txt", logsPath, 68); // todo : REMOVE THIS*/
/*	print("unique file name is %s\n", logFileName);*/
/*	/**/*/
/*	// create log file*/
/*	FIL logfile;*/
/*	// todo : uncomment these two lines*/
/*	/*f_open(&logfile, logFileName, FA_CREATE_ALWAYS);*/*/
/*	/*f_close(&logfile);*/*/
/**/
/*	// write log message*/
/*	f_open(&logfile, logFileName, FA_OPEN_APPEND | FA_WRITE);*/
/*	/*f_write(&logfile, "\n", 1, NULL);*/*/
/*	/**/*/
/*	/*const char toWrite[50] = "One of these\n";*/*/
/*	/*int writeResponse = f_write(&logfile, toWrite, sizeof(toWrite), NULL);*/*/
/*	/**/*/
/*	/*f_close(&logfile);*/*/
/*	/**/*/
/*	/*// todo: remove [for testing/verification]*/*/
/*	/*f_open(&logfile, logFileName, FA_READ);*/*/
/*	f_lseek(&logfile, 0);*/
/*	print("Reading a file:");*/
/*	char line[1024];*/
/*	print("\n----------------------------\n");*/
/*	while (f_gets(line, sizeof line, &logfile)) {*/
/*		print("%s", line);*/
/*	}*/
/*	print("\n----------------------------\n");*/
/**/
/*	f_close(&logfile);*/
/**/
/*	//    FIL file;*/
/*	//    print("Creating file\n");*/
/*	//    FRESULT open_file_res = f_open(&file, "b.txt", FA_OPEN_EXISTING |*/
/*	//    FA_WRITE); if (open_file_res != FR_OK) { 	print("Failed to open:*/
/*	//    %d\n", open_file_res);*/
/*	//        goto UNMOUNT;*/
/*	//    } else {*/
/*	//    	print("File created!\n");*/
/*	////    	int writtenAmount = f_printf(&file, "Testing writing into a*/
/*	/// file\0");*/
/*	//    	char toWrite[1024] = "main contnet 123123123123\ncontent on new*/
/*	//    line @gmail.com\nagain with Gumption\n"; 	print("Attempting to*/
/*	//    write the following to the file:");*/
/*	//    print("\n----------------------------\n"); 	print(toWrite);*/
/*	//    	print("\n----------------------------\n");*/
/*	//*/
/*	//    	int output = 0;*/
/*	//    	int result = f_write(&file, toWrite, sizeof(toWrite), &output);*/
/*	//*/
/*	//    	print("Write responded with: %d\n", result);*/
/*	//    	f_close(&file);*/
/*	//    }*/
/*	//*/
/*	//    print("\n");*/
/*	//*/
/*	//    print("Opening file for reading\n");*/
/*	//    open_file_res = f_open(&file, "b.txt", FA_READ);*/
/*	//    if (open_file_res != FR_OK) {*/
/*	//    	print("Failed to open file: %d\n", open_file_res);*/
/*	//    	goto UNMOUNT;*/
/*	//    } else {*/
/*	//    	print("File opened!\n");*/
/*	//*/
/*	//    	char line[1024];*/
/*	//    	int readBytes = 0;*/
/*	//*/
/*	//    	print("Reading a file:");*/
/*	//    	print("\n----------------------------\n");*/
/*	//        while (f_gets(line, sizeof line, &file)) {*/
/*	//        	print("%10s", line);*/
/*	//        }*/
/*	//        print("\n----------------------------\n");*/
/*	//*/
/*	//    }*/
/*	//*/
/*	//    print("\n");*/
/*	//*/
/*	//    list_dir("");*/
/**/
/*UNMOUNT:*/
/*	print("----- STOPPING SDMMC TESTING -----\n");*/
/*	f_mount(0, "", 0);*/
/**/
/*	for (;;) {*/
/*		/* empty */*/
/*	}*/
/**/
/*	vTaskDelete(NULL);*/
/*}*/
