// Conditional define is the same as #pragma once, but pragma is technically nonstandard so this is done just to be safe
#ifndef DRIVE_H
#define DRIVE_H

// Config
#define DRIVE_RATIO ratio6_1
#define DRIVE_MOTOR_SIDE_COUNT 4

#define WHEEL_DIAM  2.75 // Wheel diameter in inches
#define MAX_SPEED   95.0 // Maximum speed for autonomous operation, 95 provides ceiling for side adjustment
#define MIN_SPEED   5.0 // Minimum speed for autonomous operations
#define TICKS_PER_DEG_K 2.3 // Constant for straight movements
#define IN          *(360.0/(WHEEL_DIAM * M_PI))*TICKS_PER_DEG_K // Inches conversion for going straight
#define DRIVE_THRESH 4.0 // Maximium drive change to be considered "driving"
#define TURN_THRESH 0.3 // Degree distance from turn target that can be defined as "done" turning
#define MAX_SETTLE_COUNT 4 // The number of 20ms ticks that a threshold must be met to define the robot as "settled"
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
float driftCorrection(float stickValue);
void singleArcadeDrive();
void tankDrive();

// NOTE: ALL PID-BASED AUTONOMOUS MOVEMENTS SHOULD AUTOMATICALLY FLIP FOR OTHER SIDE

// PID autonomous
int leftControl();
int rightControl();
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
void calibrate();
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

// Calibration
void tuneMotorKFS();
void tuneMotorKFF();

#endif
