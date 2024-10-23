#include <simulink_handler.h>

static RT_MODEL rtM_;
static RT_MODEL *const rtMPtr = &rtM_; /* Real-time model */
static DW rtDW;                        /* Observable states */
static X rtX;                          /* Observable continuous states */
static XDis rtXDis;                    /* Continuous states Disabled */

/* '<Root>/thisIsAnInput' */
real_T rtU_thisIsAnInput;

/* '<Root>/thisIsAnOutput' */
real_T rtY_thisIsAnOutput;


//Functions
/*
 * Associating rt_OneStep with a real-time clock or interrupt service routine
 * is what makes the generated code "real-time".  The function rt_OneStep is
 * always associated with the base rate of the model.  Subrates are managed
 * by the base rate from inside the generated code.  Enabling/disabling
 * interrupts and floating point context switches are target specific.  This
 * example code indicates where these should take place relative to executing
 * the generated code step function.  Overrun behavior should be tailored to
 * your application needs.  This example simply sets an error status in the
 * real-time model and returns from rt_OneStep.
 */

static void rt_OneStep(RT_MODEL *const rtM)
{
  static boolean_T OverrunFlag = false;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag) {
    rtmSetErrorStatus(rtM, "Overrun");
    return;
  }

  OverrunFlag = true;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model */
  test_codegen_step(rtM, rtU_thisIsAnInput, &rtY_thisIsAnOutput);

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = false;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}


void updateModelTaskInit()
{
	RT_MODEL *const rtM = rtMPtr;

	/* Pack model data into RTM */
	rtM->dwork = &rtDW;
	rtM->contStates = &rtX;
	rtM->contStateDisabled = &rtXDis;

	/* Initialize model */
	test_codegen_initialize(rtM);

}

void updateModelTask(void *arguments)
{

	for(;;)
	{
		rt_OneStep(rtM);
		vTaskDelayUntil(200); //base model step is 0.2s
	}

}


