#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;


task left_drive;
task right_drive;
void pre_auton(void) {
    vexcodeInit();

    left_drive = task(leftControl);
    right_drive = task(rightControl);
}

void usercontrol(void) {
    // User control code here, inside the loop
    while (1) {
        driveOp();
        intakeOp(); // Intake operator control
        hooksOp(); // Lift operator control

        // Allow running auto from the controller if not in competition
        if (!Competition.isCompetitionSwitch()) {
        if (Controller.ButtonLeft.pressing()) {}
            runAuto();
        }

        wait(10, msec);
    }

    left_drive.stop();
    right_drive.stop();
    resetDrive();
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}