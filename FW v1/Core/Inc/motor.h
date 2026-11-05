#include "stdint.h"

typedef enum
{
    motorReleased = 0,    
    motorRunning,   
}motor_state_t;

typedef struct
{
    motor_state_t State;
    int32_t CurrentSpeed; //RPM
    int32_t SpeedSetPoint; //RPM
    int32_t SpeedChange; //RPM/s
    uint16_t CurrentARR;
}motor_t;
