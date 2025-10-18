//import OCV, soc, R0, I, Inom
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

//assumes temps stored as rounded integers
float gt(uint16_t cell_temp_lowest) {
    if (0 < cell_temp_lowest && cell_temp_lowest <= 15) return (float)cell_temp_lowest / 15; // cold stop 0
    if (15 < cell_temp_lowest && cell_temp_lowest <= 45) return 1; // cold full 15
    if (45 < cell_temp_lowest && cell_temp_lowest < 60) return (60 - cell_temp_lowest) / 15; // hot full 45
    return 0;
}

float gz(uint32_t soc) {
    if (0.02 < soc && soc <= 0.05) return (soc - 0.02)/0.03; // low ramp 0.02
    if (0.05 < soc && soc <= 0.9) return 1; // low full 0.05
    if (0.9 < soc && soc <= 0.97) return (0.97 - soc)/0.07; // high ramp 0.9
    return 0;
}

// need proper units/scaling principles for SoP calculation
uint32_t SoPcalc(uint32_t soc, uint16_t cell_temp_lowest, float R0, float Inom) {
    float ocv = OCV(soc, cell_temp_lowest);
    float Ichgv = fmax(0, (600-ocv)/R0);
    float Ichg = fmin(Ichgv, gt(cell_temp_lowest)*gz(soc)*Inom);
    float Vchg = ocv + Ichg * R0;
    return -Vchg*Ichg;
}