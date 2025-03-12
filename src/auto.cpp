#include "vex.h"

task straightTask;
task turnTask;
task intakeTask;
task left_drive;
task right_drive;

// center of bot starts 7" from the wall
void zionsvilleSkills_NoAllyStake() {
    straightTask = task(straightPID);
    turnTask = task(turnPID);
    intakeTask = task(autoIntake);
    left_drive = task(leftControl);
    right_drive = task(rightControl);

    disableIntake();
    drive(-17 IN);
    turn(333.4); // turn 26.6 deg left
    drive(-24 IN);
    grabMogo();
    drive(-3 IN);
    turn(180.0);
    dropIntake();
    wait(50, msec);
    enableIntake();
    drive(24 IN);
    turn(315.0);
    drive(34 IN);
    turn(45.0);
    drive(34 IN);
    wait(250, msec);
    drive(30 IN);
    wait(250, msec);
    drive(-24 IN);
    turn(225.0);
    drive(-16 IN);
    dropMogo();

    straightTask.stop();
    turnTask.stop();
    intakeTask.stop();
    left_drive.stop();
    right_drive.stop();
}

void zionsvilleSkills_AllyStake() {
    enableIntake();
    wait(500, msec);
    dropIntake();
    drive(6 IN);
    disableIntake();
    turn(296.6);
    drive(-51 IN);
    grabMogo();
    drive(-3 IN); // This point fwd is the same as normal other than no intake drop after next turn
    turn(180.0);
    enableIntake();
    drive(24 IN);
    turn(315.0);
    drive(34 IN);
    turn(45.0);
    drive(34 IN);
    wait(200, msec);
    drive(30 IN);
    wait(200, msec);
    drive(-24 IN);
    turn(225.0);
    drive(-16 IN);
    dropMogo();
}

void test() {    
    straightTask = task(straightPID);
    turnTask = task(turnPID);
    intakeTask = task(autoIntake);
    left_drive = task(leftControl);
    right_drive = task(rightControl);

    disableIntake();
    calibrate();

    // tuneMotorKFS(TURN);
    // tuneMotorKFF(TURN);

    // for (int i=0; i<4; i++) {
    //     drive(12 IN);
    //     turn(90*(i+1));    
    // }

    // Rapid turn test
    // turn(90);
    // turn(80); // Ensure we don't stall on small angles
    // turn(0);
    // turn(180);
    // turn(0);
    // turn(270);
    // turn(90);
    // turn(270.0);

    // enableIntake();
    // wait(1000, msec);
    // disableIntake();
    
    straightTask.stop();
    turnTask.stop();
    intakeTask.stop();
    left_drive.stop();
    right_drive.stop();
}

// Run the desired auto using auton selector

void runAuto() {
    zionsvilleSkills_NoAllyStake();
}

void autonomous(void) {
    runAuto();
}
