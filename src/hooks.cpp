#include "vex.h"

// Motors
motor hooks = motor(HOOKS, ratio6_1, true);

void moveHooks(float pct) {
    hooks.spin(fwd, pct*120, voltageUnits::mV);
}

void hooksOp() {
    if (Controller.ButtonR1.pressing()) {
        moveHooks(100);
    } else if (Controller.ButtonR2.pressing()) {
        moveHooks(-60);
    } else {
        hooks.stop();
    }
}