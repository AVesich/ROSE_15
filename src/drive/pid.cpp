#include "vex.h"

// PID Superclass
// Private
float PID::clamp(float target, float min, float max) {
    if (target <= min)
        return min;
    if (target >= max)
        return max;
    return target;
}

float PID::slew(float prev_value, float target_value) { // Limits accel + max speed    
    if ((prev_value < target_value) && (prev_value >= 0)) { // If accelerating
        if ((target_value - prev_value) >= accel_step) // Limit step to slew if accelerating too fast
            target_value = prev_value+accel_step;
    }

    if (target_value > max_output)
        target_value = max_output;
    else if (target_value < -max_output)
        target_value = -max_output;

    return target_value;
}

// Public
void PID::setAccelStep(float new_step) {
    accel_step = new_step;
}

void PID::reset() {
    prev_err = 0;
    total_err = 0;
    prev_output = 0;
}

float PID::computeValue(float err) {
    total_err += (prev_err-err);

    float result = kP*err + kI*(total_err) + kD*(err-prev_err);
    prev_err = err;
    if (slew_enabled) {
        result = slew(prev_output, result);
    }

    prev_output = result;
    return result;
}
