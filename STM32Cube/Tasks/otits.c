#ifdef TEST_MODE
#include "otits.h"

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"

//*****************************************************************************/
// statics
//*****************************************************************************/

// The current test to run
static int currentTestId = 0;
// Array of tests registered to run
static Otits_Test tests[MAX_NUM_TESTS] = {};
// This should never exceed `MAX_NUM_TESTS`
static int numTestsRegistered = 0;

static char* testOutcomeStrings[TEST_OUTCOME_ENUM_MAX] = {
		"PASS",
		"FAIL",
		"TIMEOUT",
		"DATA ERR",
		"UNTESTED",
};

static char* testSourceStrings[TEST_SOURCE_ENUM_MAX] = {
	    "CAN_HANDLER",
	    "CONTROLLER",
	    "FLIGHT_PHASE",
	    "HEALTH",
		"LOGGER",
		"MILLIS",
	    "OTITS",
		"SDMMC",
		"STATE_EST",
		"TRAJ",
		"VN",
		"DEFAULT",
};

/**
 * Run and store results of a test
 */
static Otits_Result_t otitsRunTest(Otits_Test* test) {
	// Run the requested test function
    printf_("[%d] running test %d...\n", (int) xTaskGetTickCount(), currentTestId);
	Otits_Result_t result = (*test->testFunctionPtr)();

	// Store results and update this test's stats using latest results
	test->latestOutcome = result.outcome;
	test->lastRunTime = xTaskGetTickCount();
	test->totalRuns++;
	if (result.outcome != TEST_OUTCOME_PASSED) {
	    printf_("[%d] TEST FAILED: %d | Info: %s\n", (int) xTaskGetTickCount(), result.outcome, result.info);
		test->runsFailed++;
	} else {
	    printf_("[%d] test passed: %d\n", (int) xTaskGetTickCount(), result.outcome);
	}

	return result;
}

/**
 * initialize things, check for issues
 */
static bool otitsInit() {
	if (numTestsRegistered == 0) {
		printf_("0 tests registered!\n");
		return true;
	} else if (numTestsRegistered > MAX_NUM_TESTS) {
		printf_("ERROR: TESTS REGISTERED %d EXCEEDS MAX %d!\n", numTestsRegistered, MAX_NUM_TESTS);
		return false;
	} else {
		printf_("%d tests successfully registered!\n", numTestsRegistered);
	}

	return true;
}

//*****************************************************************************/
// external interface
//*****************************************************************************/

/**
 * register a test to be run
 */
bool otitsRegister(Otits_Test_Function_t* testFunctionPtr, OtitsSource_e source) {
	// ensure not overflowing array
	if (numTestsRegistered == MAX_NUM_TESTS) {
		printf_("ERROR: CANNOT REGISTER MORE TESTS %d THAN MAX_NUM_TESTS!\n", numTestsRegistered);
		return false;
	}

	// add test things to array
	tests[numTestsRegistered].id = numTestsRegistered;
	tests[numTestsRegistered].lastRunTime = 0;
	tests[numTestsRegistered].totalRuns = 0;
	tests[numTestsRegistered].runsFailed = 0;
	tests[numTestsRegistered].source = source;
	tests[numTestsRegistered].testFunctionPtr = testFunctionPtr;
	tests[numTestsRegistered].latestOutcome = TEST_OUTCOME_UNTESTED;

	numTestsRegistered++;
	printf_("otits registered test %d\n", numTestsRegistered);
	return true;
}

/**
 * Task running one otit test each cycle
 */
void otitsTask(void *arg) {
	otitsInit();
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

    while (1) {
    	if (numTestsRegistered > 0) {
    		otitsRunTest(&tests[currentTestId]);

            // TODO: if test timed out, ...

            // continue to next test
            currentTestId = (currentTestId + 1) % numTestsRegistered;
    	}

        vTaskDelayUntil(&xLastWakeTime, 99);
    }
}

#endif
