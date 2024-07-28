#ifndef CONTROLLER_LIB_H_
#define CONTROLLER_LIB_H_

#include <stdbool.h>

// These controller values assume error of apogee in metres, and using time
// in seconds
#define CONTROLLER_GAIN_P 0.00005
#define CONTROLLER_GAIN_I 0.001
#define CONTROLLER_GAIN_D 0.0
#define CONTROLLER_MAX_EXTENSION 1.0
#define CONTROLLER_MIN_EXTENSION 0.0
#define CONTROLLER_I_SATMAX 10.0

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
     * Whether a call to control has been made.
     */
    bool begun;
} ControllerState;

/**
 * Initializes the state of the controller.
 */
void controller_state_init(ControllerState* state);

/**
 * Applies and updates the controller. Returns the value of PID terms summed.
 */
float control(ControllerState* state, float error);

#endif