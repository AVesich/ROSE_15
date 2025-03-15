#ifndef PID_H
#define PID_H

class PID {
    private:
    // Config
    float kP;
    float kI;
    float kD;
    float slew_enabled;
    float accel_step;
    float max_output;

    // Rolling values
    float prev_output; // for slew
    float prev_err;
    float total_err;
    float clamp(float target, float min, float max);
    float slew(float prev_value, float target_value);

    public:
    PID (float p, float i, float d, float max, bool slew = false) {
        kP = p;
        kI = i;
        kD = d;
        max_output = max;
        slew_enabled = slew;
        prev_output = 0;
    }
    void setAccelStep(float new_step);
    void reset(); // Clear values between moves
    float computeValue(float err);
};

#endif