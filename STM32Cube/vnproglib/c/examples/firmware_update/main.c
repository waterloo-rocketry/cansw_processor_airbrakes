#include <stdio.h>

/* Include files needed to do firmware updates. */
#include "vn/sensors.h"

int processErrorReceived(char* errorMessage, VnError errorCode);

/*
	Firmware Update Protocol
	1. Switch to target processor
	2. Enter Update Mode in Bootloader
	3. Set Baud Rate
	4. Verify Acknowledgement
	5. Send firmware update one record at a time
	6. Wait for Acknowledgement before sending next record
	7. Exit Update Mode in Bootloader
	8. Switch to next target processor or return back to the NAV processor
*/

int one_call_method_for_the_VN300(void)
{

	VnSensor vs;
	char modelNumber[30];
	char firmwareVersion[30];
	VnError error;

	/* This example walks through using the VectorNav C Library to do a firmware update on the different processors of the VN300 using the firmwareUpdate method. */

	 /* First determine which COM port your sensor is attached to and update the
	  * constant below. Also, if you have changed your sensor from the factory
	  * default baudrate of 115200, you will need to update the baudrate
	  * constant below as well. */
	const char SENSOR_PORT[] = "COM4";	/* Windows format for physical and virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS1"; */ /* Linux format for physical serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyUSB0"; */ /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS0"; */ /* CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1. */
	const uint32_t SENSOR_BAUDRATE = 115200;
	const uint32_t FIRMWARE_UPDATE_BAUDRATE = 115200; /* This can be different than the sensor baudrate, recommend 115200 or slower for reliability */
	const char VN300_NAVFirmwareUpdate[] = "../../FirmwareUpdates/VN300_NAV_v0.5.0.0.vnx";
	const char VN300_GPSFirmwareUpdate[] = "../../FirmwareUpdates/VN300_GPS_v0.5.0.0.vnx";


	/* We first need to initialize our VnSensor structure. */
	printf("Initialize VN Sensor.\n");
	VnSensor_initialize(&vs);

	/* Now connect to our sensor. */
	printf("Connect to VN Sensor.\n");
	if ((error = VnSensor_connect(&vs, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
		return processErrorReceived("Error connecting to sensor.", error);

	/* Let's query the sensor's model number. */
	printf("Query VN Sensor Model.\n");
	if ((error = VnSensor_readModelNumber(&vs, modelNumber, sizeof(modelNumber))) != E_NONE)
		return processErrorReceived("Error reading model number.", error);
	printf("Model Number: %s\n", modelNumber);

	/* Let's query the sensor's firmware version. */
	printf("Query VN Sensor Firmware Version.\n");
	if ((error = VnSensor_readFirmwareVersion(&vs, firmwareVersion, sizeof(firmwareVersion))) != E_NONE)
		return processErrorReceived("Error reading firmware version.", error);
	printf("Firmware Version: %s\n", firmwareVersion);

	/* Let's update the firmware on the NAV processor. */
	printf("NAV Firmware Update VN Sensor with %s.\n", VN300_NAVFirmwareUpdate);
	if ((error = VnSensor_firmwareUpdate(&vs, FIRMWARE_UPDATE_BAUDRATE, VN300_NAVFirmwareUpdate)) != E_NONE)
		return processErrorReceived("Error updating firmware.", error);

	/* Let's switch to the target processor to update. */
	printf("Switch processors on VN Sensor.\n");
	if ((error = VnSensor_switchProcessors(&vs, VNPROCESSOR_GPS, modelNumber, firmwareVersion)) != E_NONE)
		return processErrorReceived("Error switching processors.", error);

	/* Let's update the firmware on the GPS processor. */
	printf("GPS Firmware Update VN Sensor with %s.\n", VN300_GPSFirmwareUpdate);
	if ((error = VnSensor_firmwareUpdate(&vs, FIRMWARE_UPDATE_BAUDRATE, VN300_GPSFirmwareUpdate)) != E_NONE)
		return processErrorReceived("Error updating firmware.", error);

	/* Now disconnect from the sensor since we are finished. */
	printf("Disconnect from VN Sensor.\n");
	if ((error = VnSensor_disconnect(&vs)) != E_NONE)
		return processErrorReceived("Error disconnecting from sensor.", error);

	/* Depending on the sensor, a power cycle may be needed instead of a reset. */
	printf("Power Cycle the sensor.\n");

	return 0;
}

int manage_each_step_for_the_VN200(void)
{
	VnSensor vs;
	char modelNumber[30];
	char firmwareVersion[30];
	VnError error;
	int RecordSize = MAXFIRMWAREUPDATERECORDSIZE;
	char record[MAXFIRMWAREUPDATERECORDSIZE];
	bool stillGoing = true;

	/* This example walks through using the VectorNav C Library to connect to
	 * and interact with a VectorNav sensor using the VnSensor structure. */

	 /* First determine which COM port your sensor is attached to and update the
	  * constant below. Also, if you have changed your sensor from the factory
	  * default baudrate of 115200, you will need to update the baudrate
	  * constant below as well. */
	const char SENSOR_PORT[] = "COM3";	/* Windows format for physical and virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS1"; */ /* Linux format for physical serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyUSB0"; */ /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS0"; */ /* CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1. */
	const uint32_t SENSOR_BAUDRATE = 115200;
	const uint32_t FIRMWARE_UPDATE_BAUDRATE = 115200; /* This can be different than the sensor baudrate, recommend 115200 or slower for reliability */
	const char VN200FirmwareUpdate[] = "../../FirmwareUpdates/VN200_NAV_v2.0.0.1.vnx";

	/* We first need to initialize our VnSensor structure. */
	printf("Initialize VN Sensor.\n");
	VnSensor_initialize(&vs);

	/* Now connect to our sensor. */
	printf("Connect to VN Sensor.\n");
	if ((error = VnSensor_connect(&vs, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
		return processErrorReceived("Error connecting to sensor.", error);

	/* Let's query the sensor's model number. */
	printf("Query VN Sensor Model.\n");
	if ((error = VnSensor_readModelNumber(&vs, modelNumber, sizeof(modelNumber))) != E_NONE)
		return processErrorReceived("Error reading model number.", error);
	printf("Model Number: %s\n", modelNumber);

	/* Let's query the sensor's firmware version. */
	printf("Query VN Sensor Firmware Version.\n");
	if ((error = VnSensor_readFirmwareVersion(&vs, firmwareVersion, sizeof(firmwareVersion))) != E_NONE)
		return processErrorReceived("Error reading firmware version.", error);
	printf("Firmware Version: %s\n", firmwareVersion);

	/* Open Firmware Update File*/
	FILE* firmwareUpdateFile = VnSensor_openFirmwareUpdateFile(VN200FirmwareUpdate);

	/* Enter the bootloader */
	if((error = VnSensor_firmwareUpdateMode(&vs, true)) != E_NONE)
		return processErrorReceived("Failed to enter firmware update mode.", error);

	/* Give the processor some time to reboot */
	VnThread_sleepMs(1000);

	/* Change the baud rate for updating the firmware */
	if((error = VnSerialPort_changeBaudrate(&(vs.serialPort), FIRMWARE_UPDATE_BAUDRATE)) != E_NONE)
		return processErrorReceived("Failed to change the firmware update baud rate.", error);

	/* Calibrate the Bootloader before sending records */
	VnSensor_calibrateBootloader(&vs);

	/* Write the firmware update file to the sensor, one record at a time */
	while (stillGoing)
	{
		stillGoing = VnSensor_getNextFirmwareUpdateRecord(firmwareUpdateFile, record, RecordSize);
		if (stillGoing)
		{
			//printf("Send: %s\n", record);
			printf(".");
			if((error = VnSensor_writeFirmwareUpdateRecord(&vs, record)) != E_BOOTLOADER_NONE)
			{
				processErrorReceived("Failed to write record to firmware.", error);
				stillGoing = false;
			}
		}
	}
	printf("\n");

	/* Switch the baud rate back to the original setting */
	if((error = VnSerialPort_changeBaudrate(&(vs.serialPort), SENSOR_BAUDRATE)) != E_NONE)
		return processErrorReceived("Failed to change baud rate back.", error);

	/* Close the firmware update file */
	VnSensor_closeFirmwareUpdateFile(firmwareUpdateFile);

	/* Exit Bootloader mode. Just sleep for 10 seconds */
	VnThread_sleepMs(10000);

	/* Do a reset */
	VnSensor_reset(&vs, true);

	/* Wait 2 seconds for the reset to finish */
	VnThread_sleepMs(2000);

	/* Now disconnect from the sensor since we are finished. */
	printf("Disconnect from VN Sensor.\n");
	if ((error = VnSensor_disconnect(&vs)) != E_NONE)
		return processErrorReceived("Error disconnecting from sensor.", error);

	/* Depending on the sensor, a power cycle may be needed instead of a reset. */
	printf("Power Cycle the sensor.\n");
	return 0;
}

int one_call_method_for_the_VN310(void)
{

	VnSensor vs;
	char modelNumber[30];
	char firmwareVersion[30];
	VnError error;
	int RecordSize = MAXFIRMWAREUPDATERECORDSIZE;
	char record[MAXFIRMWAREUPDATERECORDSIZE];
	bool stillGoing = true;

	/* This example walks through using the VectorNav C Library to connect to
	 * and interact with a VectorNav sensor using the VnSensor structure. */

	 /* First determine which COM port your sensor is attached to and update the
	  * constant below. Also, if you have changed your sensor from the factory
	  * default baudrate of 115200, you will need to update the baudrate
	  * constant below as well. */
	const char SENSOR_PORT[] = "COM6";	/* Windows format for physical and virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS1"; */ /* Linux format for physical serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyUSB0"; */ /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS0"; */ /* CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1. */
	const uint32_t SENSOR_BAUDRATE = 115200;
	const uint32_t FIRMWARE_UPDATE_BAUDRATE = 115200; /* This can be different than the sensor baudrate, recommend 115200 or slower for reliability */
	const char VN310_NAVFirmwareUpdate[] = "../../FirmwareUpdates/VN310_NAV_v0.8.1.0.vnx";
	const char VN310_GPSFirmwareUpdate[] = "../../FirmwareUpdates/VN310_GPS_v0.8.1.0.vnx";
	const char VN310_IMUFirmwareUpdate[] = "../../FirmwareUpdates/VN310_IMU_v2.5.1.0.vnx";

	bool updateNAV = true;
	bool updateGPS = true;
	bool updateIMU = true;


	/* We first need to initialize our VnSensor structure. */
	printf("Initialize VN Sensor.\n");
	VnSensor_initialize(&vs);

	/* Now connect to our sensor. */
	printf("Connect to VN Sensor.\n");
	if ((error = VnSensor_connect(&vs, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
		return processErrorReceived("Error connecting to sensor.", error);

	/* Let's query the sensor's model number. */
	printf("Query VN Sensor Model.\n");
	if ((error = VnSensor_readModelNumber(&vs, modelNumber, sizeof(modelNumber))) != E_NONE)
		return processErrorReceived("Error reading model number.", error);
	printf("Model Number: %s\n", modelNumber);

	/* Let's query the sensor's firmware version. */
	printf("Query VN Sensor Firmware Version.\n");
	if ((error = VnSensor_readFirmwareVersion(&vs, firmwareVersion, sizeof(firmwareVersion))) != E_NONE)
		return processErrorReceived("Error reading firmware version.", error);
	printf("Firmware Version: %s\n", firmwareVersion);

	if (updateNAV)
	{
		/* Let's update the firmware on the NAV processor. */
		printf("NAV Firmware Update VN Sensor with %s.\n", VN310_NAVFirmwareUpdate);
		if ((error = VnSensor_firmwareUpdate(&vs, FIRMWARE_UPDATE_BAUDRATE, VN310_NAVFirmwareUpdate)) != E_NONE)
			return processErrorReceived("Error updating firmware.", error);

		/* Need to switch back to NAV befor we leave*/
		printf("Switch to NAV processor.\n");
		if ((error = VnSensor_switchProcessors(&vs, VNPROCESSOR_NAV, modelNumber, firmwareVersion)) != E_NONE)
			return processErrorReceived("Error switching processors.", error);
	}

	if (updateGPS)
	{
		/* Let's switch to the target processor to update. */
		printf("Switch to GPS processor.\n");
		if ((error = VnSensor_switchProcessors(&vs, VNPROCESSOR_GPS, modelNumber, firmwareVersion)) != E_NONE)
			return processErrorReceived("Error switching processors.", error);

		/* Let's update the firmware on the GPS processor. */
		printf("GPS Firmware Update VN Sensor with %s.\n", VN310_GPSFirmwareUpdate);
		if ((error = VnSensor_firmwareUpdate(&vs, FIRMWARE_UPDATE_BAUDRATE, VN310_GPSFirmwareUpdate)) != E_NONE)
			return processErrorReceived("Error updating firmware.", error);

		/* Need to switch back to NAV since we cannot switch directly to IMU from GPS. */
		printf("Switch to NAV processor.\n");
		if ((error = VnSensor_switchProcessors(&vs, VNPROCESSOR_NAV, modelNumber, firmwareVersion)) != E_NONE)
			return processErrorReceived("Error switching processors.", error);
	}

	if (updateIMU)
	{
		/* Let's switch to the target processor to update. */
		printf("Switch to IMU processor.\n");
		if ((error = VnSensor_switchProcessors(&vs, VNPROCESSOR_IMU, modelNumber, firmwareVersion)) != E_NONE)
			return processErrorReceived("Error switching processors.", error);

		/* Let's update the firmware on the GPS processor. */
		printf("IMU Firmware Update VN Sensor with %s.\n", VN310_IMUFirmwareUpdate);
		if ((error = VnSensor_firmwareUpdate(&vs, FIRMWARE_UPDATE_BAUDRATE, VN310_IMUFirmwareUpdate)) != E_NONE)
			return processErrorReceived("Error updating firmware.", error);

		/* Need to switch back to NAV befor we leave*/
		printf("Switch to NAV processor.\n");
		if ((error = VnSensor_switchProcessors(&vs, VNPROCESSOR_NAV, modelNumber, firmwareVersion)) != E_NONE)
			return processErrorReceived("Error switching processors.", error);
	}
	/* Now disconnect from the sensor since we are finished. */
	printf("Disconnect from VN Sensor.\n");
	if ((error = VnSensor_disconnect(&vs)) != E_NONE)
		return processErrorReceived("Error disconnecting from sensor.", error);

	return 0;
}


int processErrorReceived(char* errorMessage, VnError errorCode)
{
	char errorCodeStr[100];
	strFromVnError(errorCodeStr, errorCode);
	printf("%s\nERROR: %s\n", errorMessage, errorCodeStr);
	return -1;
}


int main(void)
{
	int retval = 0;

	printf("********************************************************************************\n");
	printf("Updating the firmware on a VN200.\n");
	printf("********************************************************************************\n");
	retval = manage_each_step_for_the_VN200();

	printf("********************************************************************************\n");
	printf("Updating the firmware on a VN300.\n");
	printf("********************************************************************************\n");
	retval = one_call_method_for_the_VN300();

	printf("********************************************************************************\n");
	printf("Updating the firmware on a VN310.\n");
	printf("********************************************************************************\n");
	retval = one_call_method_for_the_VN310();

	return retval;
}