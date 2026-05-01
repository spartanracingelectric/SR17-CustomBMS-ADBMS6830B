/**
 * @file    soc.c
 * @brief   State-of-charge (SoC) initialization and coulomb counting for the BMS.
 */
#include "soc.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc.h"
#include "charger.h"
#include "eeprom.h"
#include "main.h"
#include "shunt.h"
#include "usart.h"

static int16_t SOC_getCapacityFromOCV(uint16_t ocvTable[][2], uint16_t cellVoltage_mV, uint16_t ocvTableLength);
static int16_t SOC_getCapacity10C(uint16_t cellVoltage_mV);
static int16_t SOC_getCapacity25C(uint16_t cellVoltage_mV);
static int16_t SOC_getCapacity40C(uint16_t cellVoltage_mV);
static void SOC_saveToEEPROM(AccumulatorData *batt);
static int32_t SOC_calculateFromOCV(AccumulatorData *batt, ModuleData *mod);

static uint32_t lastSavedSoc_uAh = 0xFFFFFFFF;

static int32_t SOC_calculateFromOCV(AccumulatorData *batt, ModuleData *mod)
{
	// Get initial voltage and temperature readings
	uint32_t initialTime_ms = HAL_GetTick();
	uint32_t currentTime_ms = HAL_GetTick();
	while (currentTime_ms - initialTime_ms <= 3000)
	{
		Module_getCellVoltages(mod);
		Module_getVoltageStats(mod);
		Accumulator_getVoltageStats(batt, mod);
		Module_getTemperatures(mod);
		Module_getTemperatureStats(mod);
		Accumulator_getTemperatureStats(batt, mod);
		currentTime_ms = HAL_GetTick();
	}

	uint32_t packVoltage_mV = batt->sumPackVoltage_cV * 10;
	uint16_t averageCellVoltage_mV = packVoltage_mV / NUM_CELLS;

	// TODO: Replace with average temp once all temp sensors read reliably
	// Determine OCV table to use based on current temperature
	uint16_t maxCellTemp_C = batt->maxCellTemp_C;

	int16_t ocvTemps_C[NUM_OCV_TABLES] = {10, 25, 40};
	int16_t minTempDiff_C = INT16_MAX;
	int16_t selectedOcvTemp_C;
	for (int i = 0; i < NUM_OCV_TABLES; ++i)
	{
		if (abs(maxCellTemp_C - ocvTemps_C[i]) < minTempDiff_C)
		{
			minTempDiff_C = abs(maxCellTemp_C - ocvTemps_C[i]);
			selectedOcvTemp_C = ocvTemps_C[i];
		}
	}
	int16_t averageCellCapacity_mAh;
	switch (selectedOcvTemp_C)
	{
	case 10:
		averageCellCapacity_mAh = SOC_getCapacity10C(averageCellVoltage_mV);
		break;
	case 25:
		averageCellCapacity_mAh = SOC_getCapacity25C(averageCellVoltage_mV);
		break;
	default:
		averageCellCapacity_mAh = SOC_getCapacity40C(averageCellVoltage_mV);
		break;
	}
	int32_t stateOfCharge_uAh = averageCellCapacity_mAh * NUM_CELLS_IN_PARALLEL * 1000; // TODO: Check
	return stateOfCharge_uAh;
}

void SOC_init(AccumulatorData *batt, ModuleData *mod)
{
	int32_t savedSoc_mAh = EEPROM_readSOC();

	if (savedSoc_mAh != INT32_MAX && savedSoc_mAh != 0)
	{
		batt->soc = savedSoc_mAh;
	}
	else
	{
		batt->soc = SOC_calculateFromOCV(batt, mod);
		EEPROM_writeSOC(batt->soc);
	}
	lastSavedSoc_uAh = batt->soc;
}

/* ===== Coulomb Counter Update (µAh) =========================================
 * SOC_updateCharge():
 *  - Gate current to 0 if HV sense ≤ 100.00 V (centivolt field ≤ 10000).
 *  - Else refresh current via SOC_updateCurrent().
 *  - Integrate discharge:
 *      ΔQ[µAh] = 1000 * I[mA] * (Δt[ms] / 3600000)
 *      soc_new = soc_old - ΔQ
 */
void SOC_updateCharge(AccumulatorData *batt, uint32_t elapsedTime_ms)
{
	int32_t current_mA = 0;

	if (isCharging)
	{
		current_mA = batt->chargerCurrent_mA;
	}
	else
	{
		Shunt_updateAccumulator(batt);
		current_mA = -1 * (batt->shuntCurrent_mA);
	}
	
	// Ensure pack is in HV and current is not noise
	if (batt->hvSensePackVoltage_cV <= SOC_MIN_HV_PACK_VOLTAGE_CV || abs(current_mA) <= SOC_MIN_CURRENT_MA)
	{
		current_mA = 0;
	}

	float delta_mAh = current_mA * (elapsedTime_ms / 3600000.0f);
	int32_t deltaCharge_uAh = (int32_t)(delta_mAh * 1000.0f);
	batt->soc += deltaCharge_uAh;

	SOC_saveToEEPROM(batt);
}

float SOC_getPercent(AccumulatorData *batt)
{
	return ((float)batt->soc / PACK_CAPACITY_UAH) * 100.0f;
}

static int16_t SOC_getCapacityFromOCV(uint16_t ocvTable[][2], uint16_t cellVoltage_mV, uint16_t ocvTableLength)
{
	// Get the two closest capacities
	uint16_t left = 0;
	uint16_t right = ocvTableLength - 1;

	// If cellVoltage_mV is out of range of ocvTable
	if (cellVoltage_mV >= ocvTable[left][0])
	{
		return ocvTable[left][1];
	}
	if (cellVoltage_mV <= ocvTable[right][0])
	{
		return ocvTable[right][1];
	}

	while (left <= right)
	{
		int mid = left + (right - left) / 2;

		if (ocvTable[mid][0] == cellVoltage_mV)
		{
			return ocvTable[mid][1];
		}

		if (ocvTable[mid][0] > cellVoltage_mV)
		{
			left = mid + 1;
		}
		else
		{
			right = mid - 1;
		}
	}

	// If cellVoltage_mV is between ocvTable[right] and ocvTable[left]
	uint16_t highVoltage_mV = ocvTable[right][0];
	uint16_t lowVoltage_mV = ocvTable[left][0];

	if ((highVoltage_mV - cellVoltage_mV) <= (cellVoltage_mV - lowVoltage_mV))
	{
		uint16_t capacity_mAh = ocvTable[right][1];
		return capacity_mAh;
	}
	else
	{
		uint16_t capacity_mAh = ocvTable[left][1];
		return capacity_mAh;
	}
}

static void SOC_saveToEEPROM(AccumulatorData *batt)
{
	// Only write to EEPROM when SOC changes by atleast 1%
	if (abs((int32_t)batt->soc - (int32_t)lastSavedSoc_uAh) >= SOC_SAVE_THRESHOLD_UAH)
	{
		EEPROM_writeSOC(batt->soc);
		lastSavedSoc_uAh = batt->soc;
	}
}

/* ===== OCV Table Binary Search ==============================================
 * SOC_searchCapacity():
 *  - Input ‘data’ is an array of {voltage_mV, capacity_mAh}.
 *  - Returns nearest capacity for the provided per-cell voltage (mV).
 */
static int16_t SOC_getCapacity10C(uint16_t cellVoltage_mV)
{
	int ocvTableLength = 201;
	uint16_t ocvTable[][2] = {
		{4176, 5000}, {4150, 4975}, {4136, 4950}, {4125, 4925}, {4116, 4900}, {4109, 4875}, {4104, 4849}, {4099, 4824}, {4095, 4799}, {4092, 4774}, {4089, 4749}, {4087, 4724}, {4085, 4699}, {4083, 4674}, {4081, 4649}, {4080, 4624}, {4079, 4598}, {4078, 4573}, {4076, 4548}, {4075, 4523}, {4074, 4498}, {4073, 4473}, {4072, 4448}, {4072, 4423}, {4071, 4398}, {4070, 4373}, {4069, 4347}, {4068, 4322}, {4067, 4297}, {4066, 4272}, {4065, 4247}, {4064, 4222}, {4063, 4197}, {4062, 4172}, {4060, 4147}, {4059, 4122}, {4057, 4096}, {4054, 4071}, {4051, 4046}, {4048, 4021}, {4044, 3996}, {4039, 3971}, {4034, 3946}, {4028, 3921}, {4022, 3896}, {4016, 3871}, {4009, 3846}, {4002, 3820}, {3994, 3795}, {3984, 3770}, {3975, 3745}, {3965, 3720}, {3956, 3695}, {3947, 3670}, {3938, 3645}, {3931, 3620}, {3924, 3595}, {3917, 3569}, {3911, 3544}, {3906, 3519}, {3901, 3494}, {3896, 3469}, {3892, 3444}, {3887, 3419}, {3883, 3394}, {3880, 3369}, {3876, 3344}, {3873, 3318}, {3869, 3293}, {3866, 3268}, {3863, 3243}, {3860, 3218}, {3856, 3193}, {3853, 3168}, {3850, 3143}, {3846, 3118}, {3843, 3093}, {3839, 3068}, {3835, 3042}, {3831, 3017}, {3827, 2992}, {3823, 2967}, {3818, 2942}, {3813, 2917}, {3808, 2892}, {3803, 2867}, {3797, 2842}, {3792, 2817}, {3785, 2791}, {3778, 2766}, {3771, 2741}, {3763, 2716}, {3756, 2691}, {3748, 2666}, {3741, 2641}, {3733, 2616}, {3725, 2591}, {3717, 2566}, {3708, 2540}, {3699, 2515}, {3690, 2490}, {3681, 2465}, {3672, 2440}, {3663, 2415}, {3653, 2390}, {3644, 2365}, {3635, 2340}, {3627, 2315}, {3621, 2290}, {3614, 2264}, {3609, 2239}, {3603, 2214}, {3597, 2189}, {3590, 2164}, {3583, 2139}, {3574, 2114}, {3564, 2089}, {3553, 2064}, {3543, 2039}, {3536, 2013}, {3529, 1988}, {3522, 1963}, {3515, 1938}, {3509, 1913}, {3502, 1888}, {3496, 1863}, {3490, 1838}, {3483, 1813}, {3477, 1788}, {3470, 1762}, {3463, 1737}, {3457, 1712}, {3450, 1687}, {3442, 1662}, {3435, 1637}, {3428, 1612}, {3420, 1587}, {3412, 1562}, {3404, 1537}, {3396, 1511}, {3387, 1486}, {3379, 1461}, {3370, 1436}, {3361, 1411}, {3352, 1386}, {3343, 1361}, {3334, 1336}, {3324, 1311}, {3315, 1286}, {3306, 1261}, {3297, 1235}, {3287, 1210}, {3278, 1185}, {3269, 1160}, {3260, 1135}, {3251, 1110}, {3242, 1085}, {3233, 1060}, {3223, 1035}, {3214, 1010}, {3205, 984}, {3196, 959}, {3187, 934}, {3177, 909}, {3168, 884}, {3158, 859}, {3148, 834}, {3138, 809}, {3128, 784}, {3117, 759}, {3107, 733}, {3096, 708}, {3085, 683}, {3073, 658}, {3062, 633}, {3050, 608}, {3038, 583}, {3026, 558}, {3014, 533}, {3002, 508}, {2989, 483}, {2975, 457}, {2962, 432}, {2948, 407}, {2934, 382}, {2919, 357}, {2904, 332}, {2888, 307}, {2872, 282}, {2855, 257}, {2836, 232}, {2816, 206}, {2795, 181}, {2771, 156}, {2745, 131}, {2716, 106}, {2684, 81}, {2649, 56}, {2610, 31}, {2566, 6}, {2515, 0}};

	return SOC_getCapacityFromOCV(ocvTable, cellVoltage_mV, ocvTableLength);
}

static int16_t SOC_getCapacity25C(uint16_t cellVoltage_mV)
{
	int ocvTableLength = 200;
	uint16_t ocvTable[][2] = {
		{4178, 5000}, {4157, 4975}, {4143, 4950}, {4131, 4924}, {4122, 4899}, {4115, 4874}, {4109, 4849}, {4105, 4823}, {4101, 4798}, {4097, 4773}, {4095, 4748}, {4092, 4722}, {4090, 4697}, {4088, 4672}, {4087, 4647}, {4085, 4621}, {4084, 4596}, {4083, 4571}, {4082, 4546}, {4080, 4520}, {4079, 4495}, {4078, 4470}, {4078, 4445}, {4077, 4419}, {4076, 4394}, {4075, 4369}, {4074, 4344}, {4073, 4318}, {4073, 4293}, {4072, 4268}, {4071, 4243}, {4070, 4217}, {4068, 4192}, {4067, 4167}, {4066, 4142}, {4064, 4116}, {4061, 4091}, {4059, 4066}, {4056, 4041}, {4052, 4016}, {4048, 3990}, {4043, 3965}, {4038, 3940}, {4033, 3915}, {4027, 3889}, {4021, 3864}, {4015, 3839}, {4007, 3814}, {3999, 3788}, {3991, 3763}, {3981, 3738}, {3972, 3713}, {3961, 3687}, {3951, 3662}, {3941, 3637}, {3933, 3612}, {3926, 3586}, {3919, 3561}, {3913, 3536}, {3908, 3511}, {3903, 3485}, {3899, 3460}, {3894, 3435}, {3890, 3410}, {3887, 3384}, {3883, 3359}, {3880, 3334}, {3876, 3309}, {3873, 3283}, {3870, 3258}, {3867, 3233}, {3863, 3208}, {3860, 3183}, {3857, 3157}, {3854, 3132}, {3850, 3107}, {3847, 3082}, {3843, 3056}, {3839, 3031}, {3835, 3006}, {3830, 2981}, {3826, 2955}, {3821, 2930}, {3816, 2905}, {3810, 2880}, {3804, 2854}, {3798, 2829}, {3791, 2804}, {3784, 2779}, {3777, 2753}, {3769, 2728}, {3762, 2703}, {3754, 2678}, {3747, 2652}, {3740, 2627}, {3732, 2602}, {3725, 2577}, {3716, 2551}, {3708, 2526}, {3699, 2501}, {3690, 2476}, {3681, 2450}, {3673, 2425}, {3664, 2400}, {3655, 2375}, {3646, 2349}, {3637, 2324}, {3629, 2299}, {3622, 2274}, {3616, 2249}, {3610, 2223}, {3605, 2198}, {3599, 2173}, {3594, 2148}, {3588, 2122}, {3582, 2097}, {3575, 2072}, {3567, 2047}, {3558, 2021}, {3549, 1996}, {3541, 1971}, {3535, 1946}, {3528, 1920}, {3522, 1895}, {3515, 1870}, {3508, 1845}, {3502, 1819}, {3495, 1794}, {3489, 1769}, {3482, 1744}, {3475, 1718}, {3468, 1693}, {3460, 1668}, {3453, 1643}, {3446, 1617}, {3438, 1592}, {3430, 1567}, {3422, 1542}, {3414, 1516}, {3405, 1491}, {3397, 1466}, {3388, 1441}, {3379, 1415}, {3370, 1390}, {3361, 1365}, {3351, 1340}, {3342, 1315}, {3333, 1289}, {3323, 1264}, {3314, 1239}, {3304, 1214}, {3295, 1188}, {3286, 1163}, {3277, 1138}, {3267, 1113}, {3258, 1087}, {3249, 1062}, {3240, 1037}, {3231, 1012}, {3222, 986}, {3213, 961}, {3203, 936}, {3194, 911}, {3184, 885}, {3175, 860}, {3165, 835}, {3155, 810}, {3144, 784}, {3134, 759}, {3123, 734}, {3112, 709}, {3101, 683}, {3089, 658}, {3078, 633}, {3066, 608}, {3054, 582}, {3042, 557}, {3030, 532}, {3017, 507}, {3005, 481}, {2992, 456}, {2979, 431}, {2966, 406}, {2953, 381}, {2939, 355}, {2925, 330}, {2910, 305}, {2895, 280}, {2879, 254}, {2861, 229}, {2843, 204}, {2823, 179}, {2800, 153}, {2776, 128}, {2749, 103}, {2719, 78}, {2685, 52}, {2649, 27}, {2608, 2}, {2513, 0}};

	return SOC_getCapacityFromOCV(ocvTable, cellVoltage_mV, ocvTableLength);
}

static int16_t SOC_getCapacity40C(uint16_t cellVoltage_mV)
{
	int ocvTableLength = 198;
	uint16_t ocvTable[][2] = {
		{4180, 5000}, {4162, 4975}, {4147, 4949}, {4136, 4924}, {4127, 4898}, {4120, 4873}, {4114, 4848}, {4109, 4822}, {4105, 4797}, {4101, 4771}, {4098, 4746}, {4096, 4720}, {4094, 4695}, {4092, 4670}, {4090, 4644}, {4088, 4619}, {4087, 4593}, {4086, 4568}, {4085, 4542}, {4084, 4517}, {4083, 4492}, {4082, 4466}, {4081, 4441}, {4080, 4415}, {4079, 4390}, {4078, 4365}, {4077, 4339}, {4077, 4314}, {4076, 4288}, {4075, 4263}, {4074, 4238}, {4073, 4212}, {4072, 4187}, {4071, 4161}, {4069, 4136}, {4067, 4110}, {4065, 4085}, {4062, 4060}, {4059, 4034}, {4055, 4009}, {4051, 3983}, {4047, 3958}, {4042, 3932}, {4037, 3907}, {4031, 3882}, {4025, 3856}, {4019, 3831}, {4012, 3805}, {4004, 3780}, {3995, 3755}, {3986, 3729}, {3976, 3704}, {3964, 3678}, {3953, 3653}, {3943, 3627}, {3935, 3602}, {3928, 3577}, {3922, 3551}, {3917, 3526}, {3912, 3500}, {3907, 3475}, {3902, 3450}, {3898, 3424}, {3894, 3399}, {3891, 3373}, {3887, 3348}, {3884, 3322}, {3880, 3297}, {3877, 3272}, {3874, 3246}, {3871, 3221}, {3868, 3195}, {3864, 3170}, {3861, 3145}, {3858, 3119}, {3854, 3094}, {3850, 3068}, {3847, 3043}, {3842, 3017}, {3838, 2992}, {3834, 2967}, {3829, 2941}, {3824, 2916}, {3817, 2890}, {3810, 2865}, {3802, 2840}, {3795, 2814}, {3788, 2789}, {3781, 2763}, {3773, 2738}, {3766, 2712}, {3758, 2687}, {3751, 2662}, {3744, 2636}, {3737, 2611}, {3729, 2585}, {3722, 2560}, {3713, 2535}, {3705, 2509}, {3696, 2484}, {3687, 2458}, {3678, 2433}, {3670, 2408}, {3661, 2382}, {3653, 2357}, {3644, 2331}, {3635, 2306}, {3628, 2280}, {3621, 2255}, {3615, 2230}, {3609, 2204}, {3604, 2179}, {3599, 2153}, {3593, 2128}, {3588, 2102}, {3581, 2077}, {3574, 2052}, {3566, 2026}, {3556, 2001}, {3548, 1975}, {3541, 1950}, {3534, 1925}, {3528, 1899}, {3521, 1874}, {3514, 1848}, {3507, 1823}, {3501, 1798}, {3494, 1772}, {3487, 1747}, {3480, 1721}, {3473, 1696}, {3465, 1670}, {3458, 1645}, {3450, 1620}, {3442, 1594}, {3435, 1569}, {3427, 1543}, {3418, 1518}, {3410, 1492}, {3401, 1467}, {3392, 1442}, {3384, 1416}, {3374, 1391}, {3365, 1365}, {3356, 1340}, {3347, 1315}, {3338, 1289}, {3329, 1264}, {3319, 1238}, {3310, 1213}, {3301, 1188}, {3292, 1162}, {3283, 1137}, {3274, 1111}, {3266, 1086}, {3257, 1060}, {3248, 1035}, {3239, 1010}, {3230, 984}, {3221, 959}, {3212, 933}, {3203, 908}, {3194, 882}, {3184, 857}, {3175, 832}, {3165, 806}, {3155, 781}, {3145, 755}, {3135, 730}, {3124, 705}, {3113, 679}, {3102, 654}, {3091, 628}, {3080, 603}, {3068, 578}, {3057, 552}, {3045, 527}, {3033, 501}, {3021, 476}, {3009, 450}, {2997, 425}, {2984, 400}, {2971, 374}, {2959, 349}, {2945, 323}, {2932, 298}, {2917, 272}, {2903, 247}, {2887, 222}, {2870, 196}, {2851, 171}, {2831, 145}, {2808, 120}, {2783, 95}, {2754, 69}, {2723, 44}, {2688, 18}, {2515, 0}};

	return SOC_getCapacityFromOCV(ocvTable, cellVoltage_mV, ocvTableLength);
}
