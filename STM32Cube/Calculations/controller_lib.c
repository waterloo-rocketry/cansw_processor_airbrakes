#include "controller_lib.h"

void controllerStateInit(ControllerState* state) {
    state->controller_term_I = 0.0f;
    state->last_error = 0.0f;
    state->last_ms = 0.0f;
    state->begun = false;
}

float updateController(ControllerState* state, float time_ms, float error) {
    float controller_term_P = error * CONTROLLER_GAIN_P;
    float controller_term_D = 0.0f;
    if (state->begun) {
        float dt = (time_ms - state->last_ms) / 1000.0;  // time delay in s
        // Trapezoidal approximation
        state->controller_term_I +=
            CONTROLLER_GAIN_I * (state->last_error + error) * 0.5 * dt;
        if (state->controller_term_I > CONTROLLER_I_SATMAX)
            state->controller_term_I = CONTROLLER_I_SATMAX;
        if (state->controller_term_I < -CONTROLLER_I_SATMAX)
            state->controller_term_I = -CONTROLLER_I_SATMAX;

        // prevent divide by 0 errors
        if (dt >= 0.0000001)
            controller_term_D =
                CONTROLLER_GAIN_D * (error - state->last_error) / dt;
    } else {
        state->begun = true;
    }
    state->last_error = error;
    state->last_ms = time_ms;

    float output =
        controller_term_P + state->controller_term_I - controller_term_D;

    return output;
}