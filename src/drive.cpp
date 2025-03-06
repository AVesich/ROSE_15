#include "vex.h"
#include <algorithm>

// Motors
motor left1 = motor(LEFT_1, DRIVE_RATIO, false);
motor left2 = motor(LEFT_2, DRIVE_RATIO, false);
motor left3 = motor(LEFT_3, DRIVE_RATIO, false);
motor left4 = motor(LEFT_4, DRIVE_RATIO, false);
motor right1 = motor(RIGHT_1, DRIVE_RATIO, false);
motor right2 = motor(RIGHT_2, DRIVE_RATIO, false);
motor right3 = motor(RIGHT_3, DRIVE_RATIO, false);
motor right4 = motor(RIGHT_4, DRIVE_RATIO, false);
motor_group leftMotors = motor_group(left1, left2, left3, left4);
motor_group rightMotors = motor_group(right1, right2, right3, right4);

// Sensors
inertial imu = inertial(IMU);

// Utility vars
bool reversed = false;
DriveMode driveMode = MANUAL;
float maxSpeed = MAX_SPEED;
float minSpeed = MIN_SPEED;
int settleCount = 0;

// Manual Control
void driveLeft(float pct) {
    pct *= 120;
    leftMotors.spin(fwd, pct, voltageUnits::mV);
}

void driveRight(float pct) {
    pct *= 120;
    rightMotors.spin(fwd, pct, voltageUnits::mV);
}

// Operator Control
float x;
float y;
void driveOp() {
    // Store the x and y joystick values to their respective variable
    x = Controller.Axis4.value();
    y = Controller.Axis3.value();
  
    // Prevent drift
    if (fabs(x) < 0.01)
      x = 0;
    if (fabs(y) < 0.01)
      y = 0;
    
    // Make the drivetrain move
    singleArcadeDrive(x, y);
}

void singleArcadeDrive(float hori, float vert) {
    driveLeft(vert + hori);
    driveRight(vert - hori);
}

void tankDrive(float left, float right) {
    driveLeft(left);
    driveRight(right);
}

// PID autonomous
// NOTE: ALL PID-BASED AUTONOMOUS MOVEMENTS SHOULD AUTOMATICALLY FLIP FOR OTHER SIDE
float straightTarget = 0.0;
float turnTarget = 0.0;
void setStraightTarget(float target) {
    driveMode = STRAIGHT;
    straightTarget = target;
    settleCount = 0;
}

void setTurnTarget(float target) {
    driveMode = TURN;
    turnTarget = target;
    settleCount = 0;
}

// Straight
const int accelSlewStep = 0;
const int deccelSlewStep = 0;
float slewSpeed(float prevTargetSpeed, float targetSpeed) { // Both applies accel limiting/deccel limiting and top/min speed
    int appliedStep;
    
    // Limit accel/deccel
    if (fabs(prevTargetSpeed) < fabs(targetSpeed)) // accel
        appliedStep = accelSlewStep;
    else 
        appliedStep = deccelSlewStep;

    if (targetSpeed > prevTargetSpeed+appliedStep)
        targetSpeed = prevTargetSpeed+appliedStep;
    else if (targetSpeed < prevTargetSpeed-appliedStep)
        targetSpeed = prevTargetSpeed-appliedStep;
    else
        targetSpeed = prevTargetSpeed;

    if (fabs(targetSpeed) > maxSpeed)
        return maxSpeed * (targetSpeed/(fabs(targetSpeed))); // Set speed to max in the desired direction
    else if (fabs(targetSpeed) < minSpeed)
        return minSpeed * (targetSpeed/(fabs(targetSpeed)));
    
    return targetSpeed;
}

int straightPID() {
    const float kP = 0.0;
    const float kI = 0.0;
    const float kD = 0.0;
    const float kAlign = 0.0;
    
    float leftTicks = 0.0;      // Left current degrees of the robot
    float rightTicks = 0.0;     // Right current degrees of the robot
    float speed = 0.0;   // Current speed of the robot
    float currErr;
    float prevErr;
    float totalErr = 0.0;

    float prevLeftTargetSpeed;
    float prevRightTargetSpeed;
    float leftTargetSpeed;
    float rightTargetSpeed;

    while(1) {
        wait(20, msec);
        if (driveMode != STRAIGHT) continue;

        // Store prev speeds
        prevLeftTargetSpeed = leftTargetSpeed;
        prevRightTargetSpeed = rightTargetSpeed;

        // Update the degrees
        leftTicks = leftDeg();
        rightTicks = rightDeg();
        float avgTicks = (leftTicks + rightTicks)/2;
        prevErr = currErr;
        currErr = straightTarget - avgTicks;
        totalErr += (prevErr-currErr);
            
        // Speed is based on proportional distance from target, total distance traveled (boost via integral), and the rate of change in speed (decel via derivative)
        speed = kP*currErr + kI*totalErr + kD*(currErr-prevErr);
        
        // Set the left and right speed
        leftTargetSpeed = speed;
        rightTargetSpeed = speed;      
        
        // Adjust sides if necessary
        // Subtract a number proportional to the amount a side is ahead when the sides are offset.
        // This prevents the issue of telling the drive to go at max speed and thus, the slower side being unable to speed up.
        if (leftTicks > rightTicks) {
            leftTargetSpeed -= ((leftTicks - rightTicks) * kAlign);
        } else if (rightTicks > leftTicks) {
            rightTargetSpeed -= ((rightTicks - leftTicks) * kAlign);
        }

        driveLeft(slewSpeed(prevLeftTargetSpeed, leftTargetSpeed));
        driveRight(slewSpeed(prevRightTargetSpeed, rightTargetSpeed));
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
    float speed = 0.0;
    float prevSpeed;

    while(1) {
        wait(20, msec);
        if (driveMode != TURN) continue;

        currAngleErr = getShortestAngleTo(reversed ? 360.0-turnTarget : turnTarget);
        currAngleErr *= (reversed ? -1 : 1);
        totAngleErr += prevAngleErr-currAngleErr;
        prevAngleErr = currAngleErr;

        // Speed is equivalent to the left side's speed since it moves with the sign of the shortest angle
        prevSpeed = speed;
        speed = kP*currAngleErr + kI*totAngleErr + kD*(currAngleErr-prevAngleErr);

        if (currAngleErr < 0) { // Negative = counterclockwise (left turn)
            driveLeft(slewSpeed(prevSpeed, speed));
            driveRight(slewSpeed(-prevSpeed, -speed));
        } else {
            driveLeft(slewSpeed(prevSpeed, speed));
            driveRight(slewSpeed(-prevSpeed, -speed));
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
void reverse() {
    reversed = !reversed;
}

void resetDrive() {
    driveMode = MANUAL; // Prevent target resets from making the bot move
    straightTarget = 0;
    turnTarget = getAngle(); // target == where we currently are
    stopDrive();
    wait(20, msec);
    tareDrive();
}

void stopDrive() {
    leftMotors.stop();
    rightMotors.stop();
}

void tareDrive() {
    leftMotors.resetPosition();
    rightMotors.resetPosition();
}

void setBrake(brakeType mode) {
    leftMotors.setStopping(mode);
    rightMotors.setStopping(mode);
}

bool hasSettled()  {
    static float lastPos = 0;
    float currPos = leftDeg();
    
    if (fabs(lastPos-currPos) < 4.0)
        settleCount++;
    else
        settleCount = 0;
    lastPos = currPos;

    return settleCount>4 ? true : false;
}

float leftDeg() { 
    return leftMotors.position(degrees);
}

float rightDeg() { 
    return rightMotors.position(degrees);
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

