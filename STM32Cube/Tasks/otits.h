#ifndef MAIN_OTITS_H_
#define MAIN_OTITS_H_
#ifdef TEST_MODE

#include <stdbool.h>
#include "stm32h7xx_hal.h"

// MAXIMUM NUMBER OF TESTS ALLOWED TO BE REGISTERED
#define MAX_NUM_TESTS 30

extern UART_HandleTypeDef huart4;
/**
 * Log data source
*/
typedef enum {
    TEST_SOURCE_CAN_HANDLER,
    TEST_SOURCE_CONTROLLER,
    TEST_SOURCE_FLIGHT_PHASE,
    TEST_SOURCE_HEALTH,
	TEST_SOURCE_LOGGER,
	TEST_SOURCE_MILLIS,
    TEST_SOURCE_OTITS,
	TEST_SOURCE_SDMMC,
	TEST_SOURCE_STATE_EST,
	TEST_SOURCE_TRAJ,
	TEST_SOURCE_VN,
	TEST_SOURCE_DEFAULT,
	TEST_SOURCE_MY2C,
	TEST_SOURCE_ICM,
	TEST_SOURCE_ENUM_MAX,
} OtitsSource_e;

/**
 * Completion status of one test run
 */
typedef enum OtitsTestOutcome_e {
	TEST_OUTCOME_PASSED,
	TEST_OUTCOME_FAILED,
	TEST_OUTCOME_TIMEOUT,
	TEST_OUTCOME_DATA_ERR,
	TEST_OUTCOME_UNTESTED,
	TEST_OUTCOME_ENUM_MAX,
} OtitsTestOutcome_e;

/**
 * Full results from a test
 */
typedef struct Otits_Result_t {
	OtitsTestOutcome_e outcome;
	char* info;
} Otits_Result_t;

/**
 * Function signature for a otit test
 */
typedef Otits_Result_t Otits_Test_Function_t(void);

/**
 * Struct describing an otit test to run
 */
typedef struct Otits_Test {
	int id;
	OtitsSource_e source;
	Otits_Test_Function_t* testFunctionPtr;
	OtitsTestOutcome_e latestOutcome;
	int totalRuns;
	int runsFailed;
	int lastRunTime;
} Otits_Test;


extern void otitsTask(void *arg);
extern bool otitsRegister(Otits_Test_Function_t* testFunctionPtr, OtitsSource_e source);
#endif
#endif /* MAIN_OTITS_H_ */

