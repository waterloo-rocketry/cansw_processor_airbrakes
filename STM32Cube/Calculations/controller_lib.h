#ifndef CONTROLLER_LIB_H_
#define CONTROLLER_LIB_H_

#include <stdbool.h>

#include "trajectory_lib.h"

/**
 * State of the controller.
 */
typedef struct {
    /**
     * Integral term of the controller, in metre-seconds
     */
    float controller_term_I;
    /**
     * The last error value of the controller.
     */
    float last_error;
    /**
     * The time at which the controller state was last calculated.
     */
    float last_ms;
    /**
     * Whether a call to updateController has been made.
     */
    bool begun;
} ControllerState;

/**
 * Parameters for the controller.
 *
 * These controller values assume error of apogee in metres, and using time
 * in seconds
 */
typedef struct {
    /**
     * Coefficients.
     */
    float kp, ki, kd;
    /**
     * Saturation of the integral term.
     */
    float i_satmax;
    /**
     * Reference extension, assumed to allow us to hit the target apogee.
     */
    float ext_ref;
} ControllerParams;

extern const ControllerParams DEFAULT_CONTROLLER_PARAMS;

/**
 * Initializes the state of the controller.
 */
void controllerStateInit(ControllerState* state);

/**
 * Applies and updates the controller. Returns the extension that should be
 * applied.
 */
float updateController(const ControllerParams* params, ControllerState* state,
                       float time_ms, float trajectory_m, float target_m);

#endif