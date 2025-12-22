#include "contactor_sense.h"
#include <stdio.h>

void ContactorSense_getContactorState(AccumulatorData *acc) 
{
    if (HAL_GPIO_ReadPin(CONTACTOR_SENSE_GPIO_Port, CONTACTOR_SENSE_Pin) == GPIO_PIN_SET)
    {
        acc->contactorState = 1; 
        printf("CONTACTOR ON\n");
    }
    else 
    {
        acc->contactorState = 0;
        printf("CONTACTOR OFF\n");
    }
}
