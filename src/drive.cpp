#include "vex.h"

// Motors
motor left1 = motor(LEFT_1, DRIVE_RATIO, false);
motor left2 = motor(LEFT_2, DRIVE_RATIO, false);
motor left3 = motor(LEFT_3, DRIVE_RATIO, false);
motor left4 = motor(LEFT_4, DRIVE_RATIO, false);
motor right1 = motor(RIGHT_1, DRIVE_RATIO, false);
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
    left_speed = pct; // Motors use PID to reach this speed in motorControl()
}

void driveRight(float pct) {
    right_speed = pct; // Motors use PID to reach this speed in motorControl()
}

// Operator Control
void driveOp() {
    drive_mode = MANUAL;
    // Make the drivetrain move
    singleArcadeDrive();
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
const float motor_kFS = 0.0; // Feedforward static is min speed to start moving
const float motor_kFF = 0.0; // Feedforward constant provides a common speed boost required across all power levels
const float motor_kP = 1.5;
// motor_kI is omitted because it shouldn't be needed and total err would need a reset upon every new target speed to avoid tot_err reaching inf
const float motor_kD = 0.1;
motor left_motors[DRIVE_MOTOR_SIDE_COUNT] = {left1, left2, left3, left4};
motor right_motors[DRIVE_MOTOR_SIDE_COUNT] = {right1, right2, right3, right4};

int motorControl(float* target, motor motors[DRIVE_MOTOR_SIDE_COUNT]) {
    float error = 0;
    float ff_baseline, actual, prev_error, speed_boost;

    while(1) {
        for (int i=0; i<DRIVE_MOTOR_SIDE_COUNT; i++) {
            wait(20, sec);
            if (drive_mode == MANUAL) {
                speed_boost = 0;
                continue; // Disable PID for driving
            }

            ff_baseline = motor_kFS + (*target)*motor_kFF;
            actual = motors[i].velocity(percentUnits::pct);
            prev_error = error;
            error = left_speed-actual;
            speed_boost = motor_kP*error + motor_kD*prev_error;
    
            motors[i].spin(fwd, 120*ff_baseline+speed_boost, voltageUnits::mV);
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
bool slewEnabled;
float straightTarget = 0.0;
float turnTarget = 0.0;
void setStraightTarget(float target) {
    drive_mode = STRAIGHT;
    straightTarget = target;
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
    const float kP = 0.0;
    const float kI = 0.0;
    const float kD = 0.0;
    const float kAlign = 0.0;
    
    float leftTicks = 0.0;      // Left current degrees of the robot
    float rightTicks = 0.0;     // Right current degrees of the robot
    float currErr;
    float prevErr;
    float totalErr = 0.0;

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
        if (leftTicks > rightTicks) {
            left_speed -= ((leftTicks - rightTicks) * kAlign);
        } else if (rightTicks > leftTicks) {
            right_speed -= ((rightTicks - leftTicks) * kAlign);
        }

        if (slewEnabled) {
            driveLeft(slewSpeed(prev_left_speed, left_speed));
            driveRight(slewSpeed(prev_right_speed, right_speed));
        } else {
            driveLeft(left_speed);
            driveRight(right_speed);
        }
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
    float currAngleErr = getShortestAngleTo(turnTarget);
    float prevAngleErr = currAngleErr;
    float totAngleErr = 0.0;

    while(1) {
        wait(20, msec);
        if (drive_mode != TURN) continue;

        currAngleErr = getShortestAngleTo(turnTarget);
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
    imu.calibrate();

    while (imu.isCalibrating()) wait(20, msec);
}

void reverse() {
    reversed = !reversed;
}

void resetDrive() {
    drive_mode = MANUAL; // Prevent target resets from making the bot move
    straightTarget = 0;
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

const float DRIVE_DONE_THRESH = 4.0; // Maximium drive change to be considered "driving"
const float TURN_DONE_THRESH = 0.3; // Degree distance from turn target that can be defined as "done" turning
const int DONE_SETTLING_COUNT = 4;
bool hasSettled()  {
    float curr_position;

    if (drive_mode == STRAIGHT) {
        curr_position = leftDeg();
        
        if (fabs(last_position-curr_position) < DRIVE_DONE_THRESH)
            settle_count++;
        else
            settle_count = 0;
            last_position = curr_position;
    } else if (drive_mode == TURN) {
        curr_position = getShortestAngleTo(turnTarget);

        if (fabs(curr_position) < TURN_DONE_THRESH)
            settle_count++;
        else 
            settle_count = 0;
    } else {
        return true;
    }

    return settle_count>DONE_SETTLING_COUNT ? true : false;
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
    deg = reversed ? 360.0-deg : deg; // Automatically flip drive when on the reversed side
    return fmod((deg-getAngle()+540.0), 360.0)-180.0;
}

// Odom
float getX() { 
    return -1;
}

float getY() {
    return -1;
}

