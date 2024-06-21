#include "sdmmc.h"
#include "otits.h"
#include <string.h>

const char *logsPath = "/LOGS"; //path to the log file directory on the SD card
char logFileName[LOG_FILE_NAME_SIZE];

Otits_Result_t test_sdmmcWriteReadFile() {
	Otits_Result_t res;
	FIL testFileW;
    FIL testFileR;
	const char *testString = "otits";
	size_t stringLen = strlen(testString);
	// open a new empty test file
	if (f_open(&testFileW, "test.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        res.outcome = TEST_OUTCOME_FAILED;
        res.info = "open fileW fail";
		return res;
	}

	// write something
	UINT bytesWritten = 0;
	if (f_write(&testFileW, testString, stringLen, &bytesWritten) != FR_OK) {
        res.outcome = TEST_OUTCOME_FAILED;
        res.info = "write fileW fail";
		return res;
	} else if (bytesWritten != stringLen) {
        res.outcome = TEST_OUTCOME_FAILED;
        res.info = "bytesWritten incorrect";
		return res;
	}

    // close the file
    if (f_close(&testFileW) != FR_OK) {
        res.outcome = TEST_OUTCOME_FAILED;
        res.info = "close fileW fail";
        return res;
    }

    // open again to check that the file was actually created
    if (f_open(&testFileR, "test.txt", FA_OPEN_EXISTING | FA_READ) != FR_OK) {
        res.outcome = TEST_OUTCOME_FAILED;
        res.info = "open fileR fail";
        return res;
    }

	// read the msg
	BYTE buf[5];
	UINT bytesRead = 0;
	if (f_read(&testFileR, buf, stringLen, &bytesRead) != FR_OK) {
        res.outcome = TEST_OUTCOME_FAILED;
        res.info = "read fileR fail";
		return res;
	} else if (strncmp(testString, buf, 5) != 0) {
	    f_close(&testFileR); // try to close it here so future tests don't get locked out
        res.outcome = TEST_OUTCOME_FAILED;
        res.info = "strcmp fail";
		return res;
	}

	// close the file again
	if (f_close(&testFileR) != FR_OK) {
        res.outcome = TEST_OUTCOME_FAILED;
        res.info = "close fileR fail";
		return res;
	}

	res.info = "";
	res.outcome = TEST_OUTCOME_PASSED;
	return res;
}

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

