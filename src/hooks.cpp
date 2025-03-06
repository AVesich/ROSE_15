#include "vex.h"

// Motors
motor hooks = motor(INTAKE, ratio6_1);

void moveHooks(float pct) {
    hooks.spin(fwd, pct*120, voltageUnits::mV);
}

void hooksOp() {
    if (Controller.ButtonL1.pressing()) {
        moveHooks(100);
    } else if (Controller.ButtonL2.pressing()) {
        moveHooks(-60);
    } else {
        hooks.stop();
    }
}