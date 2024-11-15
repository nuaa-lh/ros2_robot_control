
#include "omega.h"
#include <stdio.h>

void openHaptics()
{

    if (dhdOpen() < 0)
    {
        printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep(1.0);
        return;
    }

    printf("%s haptic device detected\n\n", dhdGetSystemName());

}

void openforce()
{
    dhdEnableForce(DHD_ON);
    dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    dhdSleep(0.5);
}

void closeHaptics()
{
    dhdClose();
}