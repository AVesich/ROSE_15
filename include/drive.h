// Conditional define is the same as #pragma once, but pragma is technically nonstandard so this is done just to be safe
#ifndef DRIVE_H
#define DRIVE_H

#define WHEEL_DIAM  4.0 // Wheel diameter in inches
#define MAX_SPEED   127.0 // Maximum speed for autonomous operations
#define MIN_SPEED   5.0 // Minimum speed for autonomous operations
#define TICKS_PER_DEG_K 1.0 // Constant for straight movements
#define IN          *(360.0/(WHEEL_DIAM * M_PI))*TICKS_PER_DEG_K // Inches conversion for going straight
#define WHEEL_SPACING  // Space between the wheels in inches

enum DriveMode {
    STRAIGHT,
    TURN,
    MANUAL // Used for special cases where we don't want the pid to be constantly running
};

// NOTE: ALL DRIVE FUNCTIONS ARE IMPLEMENTED USING VOLTAGE.
// THEY TAKE PCT AS AN INPUT FOR CLARITY AND EASE OF CALCULATIONS.
// THIS IS BECAUSE VOLTAGE DISABLES VEX PID AND IMPROVES RESPONSIVENESS TO CUSTOM PID AND DRIVER INPUT AS A RESULT.
// Manual Control
void driveLeft(float pct);
void driveRight(float pct);
void driveOp();
void singleArcadeDrive(float hori, float vert);
void tankDrive(float left, float right);

// NOTE: ALL PID-BASED AUTONOMOUS MOVEMENTS SHOULD AUTOMATICALLY FLIP FOR OTHER SIDE

// PID autonomous
// Straight
int straightPID();
void drive(float targetDeg);
// Turn
int turnPID();
void turn(float targetHeading);
// Arc
void arc(float radius, float deg);
// SCurve
void scurve(float xDist, float yDist); // The direction provided is the way it ends up offset horizontally

// Odom autonomous
void driveTo(float x, float y);

// Utility methods
void reverse();
void resetDrive();
void stopDrive();
void tareDrive();
void setBrake(brakeType mode);
bool hasSettled();
float leftDeg();
float rightDeg();
// Odom
float getX();
float getY();
float getAngle();
float getShortestAngleTo(float deg);

#endif
