#include "vex.h"

// Motors
motor intake = motor(INTAKE, ratio6_1);

void moveIntake(float pct) {
    intake.spin(fwd, pct*120, voltageUnits::mV);
}

bool lowered = false;
void liftIntake() {
    if (!lowered) return;

    // intakeSolenoid.close();
    lowered = false;
}

void dropIntake() {
    if (lowered) return;

    // intakeSolenoid.open();
    lowered = true;
}

void intakeOp() {
    if (Controller.ButtonR1.pressing()) {
        moveIntake(100);
    } else if (Controller.ButtonR2.pressing()) {
        moveIntake(-60);
    } else {
        intake.stop();
    }

    if (Controller.ButtonUp.pressing()) {
        liftIntake();
    } else if (Controller.ButtonDown.pressing()) {
        dropIntake();
    }
}

// Autonomous
bool stopIntake = false;
int autoIntake() {
    while(1) {
        wait(20, msec);

        moveIntake(100);
        moveHooks(80);

        if (stopIntake) continue;
    }

    return 0;
}

void enableIntake() {
    stopIntake = false;
}

void disableIntake() {
    stopIntake = true;
}