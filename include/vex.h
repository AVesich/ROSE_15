#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

// Important namespaces
using namespace vex;
using namespace std;

// Custom inclusions
#include "robot-config.h"
#include "deviceConfig.h"
#include "drive.h"
#include "intake.h"
#include "hooks.h"
#include "mogo.h"
#include "macros.h"
#include "auto.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)