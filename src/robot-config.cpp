#include "vex.h"

using namespace vex;

// Devices
brain Brain;
controller Controller = controller(primary);
pneumatics mogoSolenoid = pneumatics(Brain.ThreeWirePort.B);

void vexcodeInit( void ) {}