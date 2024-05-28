#include "sdmmc.h"

const char *logsPath = "/LOGS";
char logFileName[LOG_FILE_NAME_SIZE];

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
	while (snprintf_(logFileName, LOG_FILE_NAME_SIZE, "%s/%d.txt", logsPath, ++nextValidFileNumber) > 0 &&
		f_stat(logFileName, NULL) == FR_OK);
}

