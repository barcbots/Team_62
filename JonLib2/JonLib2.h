#pragma systemFile

#pragma platform(VEX2)
#pragma competitionControl(competition)

#include "Includes/VEX_Competition_Includes_JON.c"

#define MOVE_TIMEOUT 1000 //timeout for "auto" moves
#define THRESHOLD_COEFF 2 //expanded size of threshold for timeout

#include "Includes/Math.h"
#include "Includes/PID.h"
#include "Includes/Movement.h"
#include "Includes/Gyro.h"

#warning "       __            __    _ __   ___ "
#warning "      / /___  ____  / /   (_) /_ |__ \\"
#warning " __  / / __ \\/ __ \\/ /   / / __ \\__/ /"
#warning "/ /_/ / /_/ / / / / /___/ / /_/ / __/ "
#warning "\\____/\\____/_/ /_/_____/_/_.___/____/ "
