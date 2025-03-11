#include "vex.h"

// Motors
motor left1 = motor(LEFT_1, DRIVE_RATIO, true);
motor left2 = motor(LEFT_2, DRIVE_RATIO, false);
motor left3 = motor(LEFT_3, DRIVE_RATIO, true);
motor left4 = motor(LEFT_4, DRIVE_RATIO, true);
motor right1 = motor(RIGHT_1, DRIVE_RATIO, true);
motor right2 = motor(RIGHT_2, DRIVE_RATIO, false);
motor right3 = motor(RIGHT_3, DRIVE_RATIO, false);
motor right4 = motor(RIGHT_4, DRIVE_RATIO, false);
motor_group left_group = motor_group(left1, left2, left3, left4);
motor_group right_group = motor_group(right1, right2, right3, right4);

// Sensors
inertial imu = inertial(IMU);

// Utility vars
bool reversed = false;
DriveMode drive_mode = MANUAL;
float max_speed = MAX_SPEED;
float min_speed = MIN_SPEED;
int settle_count = 0;
float last_position = 0; // Most recent drive position, used purely in isSettled() and resetDrive()

// Speed vars
float prev_left_speed = 0.0, prev_right_speed = 0.0;
float left_speed= 0.0, right_speed = 0.0;

// Manual Control
void driveLeft(float pct) {
    left_group.spin(fwd, 120*pct, voltageUnits::mV);
}

void driveRight(float pct) {
    right_group.spin(fwd, 120*pct, voltageUnits::mV);
}

// Operator Control
void driveOp() {
    drive_mode = MANUAL;
    // Make the drivetrain move
    tankDrive();
}

float driftCorrection(float stickValue) {
    return (fabs(stickValue) < 0.1) ? 0.0 : stickValue;
}

void singleArcadeDrive() {
    float vert = driftCorrection(Controller.Axis3.value());
    float hori = driftCorrection(Controller.Axis4.value());

    driveLeft(vert + hori);
    driveRight(vert - hori);
}

void tankDrive() {
    float left = driftCorrection(Controller.Axis3.value());
    float right = driftCorrection(Controller.Axis2.value());

    driveLeft(left);
    driveRight(right);
}

// Motor Control, uses feed-forward + PID
const float motor_kFS = 5.0; // Feedforward static is min speed to start moving
const float motor_kFF = 0.3; // Feedforward constant provides a common speed boost required across all power levels
const float motor_kP = 0.25;
// motor_kI is omitted because it shouldn't be needed and total err would need a reset upon every new target speed to avoid tot_err reaching inf
const float motor_kD = 0.1;
motor* left_motors[DRIVE_MOTOR_SIDE_COUNT] = {&left1, &left2, &left3, &left4};
motor* right_motors[DRIVE_MOTOR_SIDE_COUNT] = {&right1, &right2, &right3, &right4};

using one_side_motors = motor*[DRIVE_MOTOR_SIDE_COUNT];
int motorControl(float* target, one_side_motors& motors) {
    float error = 0;
    float ff_baseline, actual, prev_error, speed_boost;

    while(1) {
        wait(20, msec);
        if (drive_mode == MANUAL) {
            continue; // Disable PID for driving
        }

        for (int i=0; i<DRIVE_MOTOR_SIDE_COUNT; i++) {
            ff_baseline = motor_kFS + (*target)*motor_kFF;
            actual = (*motors[i]).velocity(percentUnits::pct);
            prev_error = error;
            error = left_speed-actual;
            speed_boost = motor_kP*error + motor_kD*prev_error;
    
            (*motors[i]).spin(fwd, 120*(ff_baseline+speed_boost), voltageUnits::mV);
        }
    }

    return 0;
}

int leftControl() {
    return motorControl(&left_speed, left_motors);
}

int rightControl() {
    return motorControl(&right_speed, right_motors);
}

// PID autonomous
// NOTE: ALL PID-BASED AUTONOMOUS MOVEMENTS SHOULD AUTOMATICALLY FLIP FOR OTHER SIDE
bool slewEnabled = false;
float straightTarget = 0.0;
float holdAngle = 0.0;
float turnTarget = 0.0;
void setStraightTarget(float target) {
    drive_mode = STRAIGHT;
    straightTarget = target;
    holdAngle = getAngle();
    settle_count = 0;
}

void setTurnTarget(float target) {
    drive_mode = TURN;
    turnTarget = target;
    settle_count = 0;
}

// Straight
int direction(float speed) {
    if (speed >= 0)
        return 1;
    else
        return -1;
}

float clamp(float target, float min, float max) {
    if (target <= min)
        return min;
    if (target >= max)
        return max;
    return target;
}

const int ACCEL_STEP = 9;
float slewSpeed(float prevTargetSpeed, float targetSpeed) { // Limits accel + max speed    
    if ((prevTargetSpeed < targetSpeed) && (prevTargetSpeed >= 0)) { // If accelerating
        if ((targetSpeed - prevTargetSpeed) >= ACCEL_STEP) // Limit step to slew if accelerating too fast
            targetSpeed = prevTargetSpeed+ACCEL_STEP;
    }

    return clamp(targetSpeed, min_speed, max_speed);
}

int straightPID() {
    const float kP = 0.1;
    const float kI = 0.0;
    const float kD = 0.05;
    const float kAlign = 3.0;
    
    float leftTicks = 0.0;      // Left current degrees of the robot
    float rightTicks = 0.0;     // Right current degrees of the robot
    float currErr;
    float prevErr;
    float totalErr = 0.0;
    float angle_error;

    while(1) {
        wait(20, msec);
        if (drive_mode != STRAIGHT) continue;

        // Update the degrees
        leftTicks = leftDeg();
        rightTicks = rightDeg();
        float avgTicks = (leftTicks + rightTicks)/2;
        prevErr = currErr;
        currErr = straightTarget - avgTicks;
        totalErr += (prevErr-currErr);
            
        // Store prev speeds
        prev_left_speed = left_speed;
        prev_right_speed = right_speed;
        
        // Speed is based on proportional distance from target, total distance traveled (boost via integral), and the rate of change in speed (decel via derivative)
        left_speed = kP*currErr + kI*totalErr + kD*(currErr-prevErr);
        right_speed = left_speed;
                
        // Adjust sides if necessary
        // Subtract a number proportional to the amount a side is ahead when the sides are offset.
        // This prevents the issue of telling the drive to go at max speed and thus, the slower side being unable to speed up.
        angle_error = getShortestAngleTo(holdAngle);
        if (angle_error > TURN_THRESH) { // Drifted to the left
            right_speed -= (angle_error * kAlign);
        } else if (angle_error < -TURN_THRESH) { // Drifted to the right
            left_speed -= (fabs(angle_error) * kAlign);
        }

        if (slewEnabled) {
            left_speed = slewSpeed(prev_left_speed, left_speed);
            right_speed = slewSpeed(prev_right_speed, right_speed);
        }

        Brain.Screen.print(left_speed);
        Brain.Screen.print(right_speed);
    }

    return 0;
}

void drive(float targetDeg) {
    resetDrive();
    setBrake(hold);
    setStraightTarget(targetDeg);
    wait(450, msec);
    while(!hasSettled()) wait(20, msec);
}

// Turn
int turnPID() {
    // Find and init deg related attributes
    const float kP = 0.0;
    const float kI = 0.0;
    const float kD = 0.0;
    float currAngleErr = getShortestAngleTo(reversed ? 360.0-turnTarget : turnTarget);
    float prevAngleErr = currAngleErr;
    float totAngleErr = 0.0;

    while(1) {
        wait(20, msec);
        if (drive_mode != TURN) continue;

        currAngleErr = getShortestAngleTo(reversed ? 360.0-turnTarget : turnTarget);
        currAngleErr *= (reversed ? -1 : 1);
        totAngleErr += prevAngleErr-currAngleErr;
        prevAngleErr = currAngleErr;

        // Speed is equivalent to the left side's speed since it moves with the sign of the shortest angle
        prev_left_speed = left_speed;
        prev_right_speed = right_speed;

        left_speed = kP*currAngleErr + kI*totAngleErr + kD*(currAngleErr-prevAngleErr);
        right_speed = -left_speed;

        if (slewEnabled) {
            driveLeft(slewSpeed(prev_left_speed, left_speed));
            driveRight(-slewSpeed(prev_right_speed, right_speed));
        } else {
            driveLeft(left_speed);
            driveRight(right_speed);
        }
    }

    return 0;
}

void turn(float targetHeading) {
    resetDrive();
    setBrake(hold);
    setTurnTarget(targetHeading);
    wait(450, msec);
    while(!hasSettled()) wait(20, msec);
}

// Arc
void arc(float radius, float deg) {

}

// SCurve // The direction provided is the way it ends up offset horizontally
void scurve(float xDist, float yDist) {

}

// Odom autonomous
void driveTo(float x, float y) {

}

// Utility methods
void calibrate() {
    drive_mode = MANUAL;
    imu.calibrate();

    while (imu.isCalibrating()) wait(20, msec);
}

void reverse() {
    reversed = !reversed;
}

void resetDrive() {
    drive_mode = MANUAL; // Prevent target resets from making the bot move
    straightTarget = 0;
    holdAngle = getAngle();
    turnTarget = getAngle(); // target == where we currently are
    last_position = 0; // Used for stopping
    stopDrive();
    wait(20, msec);
    tareDrive();
}

void stopDrive() {
    left_speed = 0;
    prev_left_speed = 0;
    right_speed = 0;
    prev_right_speed = 0;
    left_group.stop();
    right_group.stop();
}

void tareDrive() {
    left_group.resetPosition();
    right_group.resetPosition();
}

void setBrake(brakeType mode) {
    left_group.setStopping(mode);
    right_group.setStopping(mode);
}

bool hasSettled()  {
    float curr_position;

    if (drive_mode == STRAIGHT) {
        curr_position = leftDeg();
        
        if (fabs(last_position-curr_position) < DRIVE_THRESH)
            settle_count++;
        else
            settle_count = 0;
            last_position = curr_position;
    } else if (drive_mode == TURN) {
        curr_position = getShortestAngleTo(reversed ? 360.0-turnTarget : turnTarget);

        if (fabs(curr_position) < TURN_THRESH)
            settle_count++;
        else 
            settle_count = 0;
    } else {
        return true;
    }

    return settle_count>MAX_SETTLE_COUNT ? true : false;
}

float leftDeg() { 
    return left_group.position(degrees);
}

float rightDeg() { 
    return right_group.position(degrees);
}

float getAngle() {
    return imu.heading();
}

float getShortestAngleTo(float deg) {
    return fmod((deg-getAngle()+540.0), 360.0)-180.0;
}

// Odom
float getX() { 
    return -1;
}

float getY() {
    return -1;
}

// Calibration
void tuneMotorKFS() {
    const float thresh = 10.0;
    float speed = 0.0;
    float dist = 0.0;

    drive_mode = MANUAL;
    while (dist < thresh) {
        speed += 0.5;
        dist = 0.0;

        driveLeft(speed);
        driveRight(speed);
        wait(500, msec);
        resetDrive();

        dist = (left_group.position(rotationUnits::deg)+right_group.position(rotationUnits::deg))/2.0;
    }

    Brain.Screen.clearScreen();
    Brain.Screen.print("Tuned motor kFS: %f", speed); // Tuned kFS
}

float leastSquareRegressionSlope(const float x_values[], const float y_values[], int num_values) {
    float x_avg = 0.0, y_avg = 0.0;
    for (int i=0; i<num_values; i++) {
        x_avg += x_values[i];
        y_avg += y_values[i];
    }
    x_avg /= num_values;
    y_avg /= num_values;

    float numerator_sum = 0.0, denominator_sum = 0.0;
    for (int i=0; i<11; i++) {
        numerator_sum += (x_values[i] - x_avg) * (y_values[i] - y_avg);
        denominator_sum += pow((x_values[i] - x_avg), 2.0);
    }

    return numerator_sum/denominator_sum;
}

void tuneMotorKFF() {
    const float velocities[10] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90}; // Stop at 90 since inability to reach perfect 100 will lead to driving forever
    float kFFs[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float speed = 0.0;

    drive_mode = MANUAL;
    for (int i=0; i<11; i++) {
        speed = velocities[i] - 0.5;

        while ((left_group.velocity(percentUnits::pct) < velocities[i]) &&
               (right_group.velocity(percentUnits::pct) < velocities[i])) {
            speed += 0.5;
            driveLeft(speed);
            driveRight(speed);
            wait(20, msec);
        }
        stopDrive();

        kFFs[i] = speed/velocities[i];
        wait(1000, msec);
    }

    // Find slope of line of best fit for overall kFF using least square method
    Brain.Screen.clearScreen();
    Brain.Screen.print("Tuned motor kFF: %f", leastSquareRegressionSlope(velocities, kFFs, 10)); // Tuned kFF
}