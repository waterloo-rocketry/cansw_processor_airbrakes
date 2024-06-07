#include "sdmmc.h"

const char *logsPath = "/LOGS"; //path to the log file directory on the SD card
char logFileName[LOG_FILE_NAME_SIZE];


//Count the number of files inside of a directory
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

//Create a unique log filename using the number of files in the log directory (liable to break if files are deleted out of order)
void initUniqueLogFileName() {
	int nextValidFileNumber = computeFolderSize(logsPath) - 1;
	while (snprintf_(logFileName, LOG_FILE_NAME_SIZE, "%s/%d.txt", logsPath, ++nextValidFileNumber) > 0 &&
		f_stat(logFileName, NULL) == FR_OK);
}

