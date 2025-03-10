#include "vex.h"

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
    if (Controller.ButtonL1.pressing()) {
        grabbing ? dropMogo() : grabMogo();
    } 
}