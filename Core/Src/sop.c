#include "main.h"
#include "sop.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//assumes temps stored as rounded integers
float gt(uint16_t cell_temp_lowest) {
    if (0 < cell_temp_lowest && cell_temp_lowest <= 15) return (float)cell_temp_lowest / 15; // cold stop 0
    if (15 < cell_temp_lowest && cell_temp_lowest <= 45) return 1; // cold full 15
    if (45 < cell_temp_lowest && cell_temp_lowest < 60) return (60 - cell_temp_lowest) / 15; // hot full 45
    return 0;
}
//add cold stop/full hot full in header

// soc currently not scaled properly for these calculations
// needs nominal_capacity to reduce to fraction form
float gz(uint32_t soc) {
    if (20 < soc && soc <= 50) return 1000*(soc - 20)/30; // low ramp 0.02
    if (50 < soc && soc <= 900) return 1000; // low full 0.05
    if (900 < soc && soc <= 970) return 1000*(970 - soc)/70; // high ramp 0.9
    return 0;
} //currently returns millifraction

// need proper units/scaling principles for SoP calculation
uint32_t SoPcalc(uint32_t soc, uint16_t cell_temp_lowest, float R0, float Inom) {
    float ocv = OCV(soc, cell_temp_lowest);
    float Ichgv = fmax(0, (600-ocv)/R0);
    float Ichg = fmin(Ichgv, gt(cell_temp_lowest)*gz(soc)*Inom);
    float Vchg = ocv + Ichg * R0;
    return -Vchg*Ichg;
}

// ignore heavy float usage (will change in accordance with given units/scaling and fixed point math)