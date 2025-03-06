#include "vex.h"

pneumatics mogoSolenoid = pneumatics(MOGO_SOLENOID);

bool grabbing = false;
void grabMogo() {
    if (grabbing) return;

    mogoSolenoid.open();
    grabbing = true;
}

void dropMogo() {
    if (!grabbing) return;

    mogoSolenoid.close();
    grabbing = false;
}

void mogoOp() {
    if (Controller.ButtonR1.pressing()) {
        grabMogo();
    } else if (Controller.ButtonR2.pressing()) {
        dropMogo();
    }
}