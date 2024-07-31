#include "controller_lib.h"

void controllerStateInit(ControllerState* state) {
    state->controller_term_I = 0.0f;
    state->last_error = 0.0f;
    state->last_ms = 0.0f;
    state->begun = false;
}

float updateController(const ControllerParams* params, ControllerState* state,
                       float time_ms, float trajectory_m, float target_m) {
    float error = target_m - trajectory_m;
    float controller_term_P = error * params->kp;
    float controller_term_D = 0.0f;
    if (state->begun) {
        float dt = (time_ms - state->last_ms) / 1000.0;  // time delay in s
        // Trapezoidal approximation
        state->controller_term_I +=
            params->ki * (state->last_error + error) * 0.5 * dt;
        if (state->controller_term_I > params->i_satmax)
            state->controller_term_I = params->i_satmax;
        if (state->controller_term_I < -params->i_satmax)
            state->controller_term_I = -params->i_satmax;

        // prevent divide by 0 errors
        if (dt >= 0.0000001)
            controller_term_D = params->kd * (error - state->last_error) / dt;
    } else {
        state->begun = true;
    }
    state->last_error = error;
    state->last_ms = time_ms;

    float output =
        controller_term_P + state->controller_term_I - controller_term_D;

    float extension = EXTENSION_REFERENCE - output;

    if (!(0.0f <= extension)) extension = 0.0f;
    if (!(extension <= 1.0f)) extension = 1.0f;

    return extension;
}