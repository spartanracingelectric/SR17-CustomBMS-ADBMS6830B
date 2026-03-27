/**
 * @file    safety.c
 * @brief   Pack/cell safety evaluation: voltage & temperature checks with hysteresis.
 *
 * This file provides:
 *  - Cell_Voltage_Fault(): per-cell UV/OV evaluation, warning/fault latching,
 *                          and fault line assertion/clear with hysteresis.
 *  - Cell_Temperature_Fault(): per-sensor high-temp evaluation with hysteresis.
 *  - (commented examples) Pack-level checks and module averages.
 *
 * Conventions:
 *  - Cell voltage units: deci-millivolts (0.1 mV). Example: 42_000 → 4.200 V.
 *  - Pack voltage fields used elsewhere (e.g., hvsens_pack_voltage) are centivolts (cV).
 *  - Temperatures are in degrees Celsius (°C).
 *  - Warning/fault summary bits are defined in safety.h.
 *
 * Hysteresis & Latching:
 *  - *_hysteresis counters debounce conditions across multiple cycles
 *    (e.g., >16) before asserting a fault bit.
 *  - *_fault_lock flags keep faults latched until the signal clears by the
 *    specified margin (FAULT_LOCK_MARGIN_*), at which point ClearFaultSignal()
 *    is invoked and bits are deasserted.
 *
 * Assumptions:
 *  - SendFaultSignal()/ClearFaultSignal() drive an external shutdown/fault line.
 *  - NUM_CELLS / NUM_THERM_TOTAL and thresholds are defined in headers.
 */
#include "safety.h"
#include "accumulator.h"
#include "gpio.h"
#include "main.h"
#include "stdio.h"

static void Safety_clearFaults(void);
static void Safety_checkPEC(int m, FaultFlags_t *module_faults, ModuleData *mod, uint32_t current_time);
static void Safety_checkOpenWire(int m, int c, uint16_t voltage, FaultFlags_t *faults, uint32_t current_time);
static void Safety_checkRedundantVoltage(int m, int c, uint16_t voltage, FaultFlags_t *faults,
										 uint16_t redundantVoltage, uint32_t current_time);
static void Safety_checkOverVoltage(int m, int c, int16_t voltage, FaultFlags_t *faults, WarningFlags_t *warns,
									uint32_t current_time);
static void Safety_checkUnderVoltage(int m, int c, int16_t voltage, FaultFlags_t *faults, WarningFlags_t *warns,
									 uint32_t current_time);
static void Safety_checkOverTemp(int m, int t, uint16_t temp, FaultFlags_t *faults, WarningFlags_t *warns,
								 uint32_t current_time);
static void Safety_checkUnderTemp(int m, int t, uint16_t temp, FaultFlags_t *faults, WarningFlags_t *warns,
								  uint32_t current_time);

/* ===== Global Latches / Hysteresis Counters =================================
 * GlobalFaults[][]:   per-module/cell fault bitfields.
 * GlobalWarnings[][]: per-module/cell warning bitfields.
 * Timer_...[][]:      per-module/cell hysteresis counters for each fault type.
 */
FaultFlags_t   GlobalFaults[NUM_MOD][NUM_CELL_PER_MOD]   = {0};
WarningFlags_t GlobalWarnings[NUM_MOD][NUM_CELL_PER_MOD] = {0};

static uint32_t Timer_RedundancyVolt[NUM_MOD][NUM_CELL_PER_MOD] = {0};
static uint32_t Timer_UnderVolt[NUM_MOD][NUM_CELL_PER_MOD]      = {0};
static uint32_t Timer_OverVolt[NUM_MOD][NUM_CELL_PER_MOD]       = {0};
static uint32_t Timer_UnderTemp[NUM_MOD][NUM_THERM_PER_MOD]     = {0};
static uint32_t Timer_OverTemp[NUM_MOD][NUM_THERM_PER_MOD]      = {0};
static uint32_t Timer_OpenWire[NUM_MOD][NUM_CELL_PER_MOD]       = {0};
static uint32_t Timer_PEC[NUM_MOD]                              = {0};

void Safety_checkFaults(AccumulatorData *batt, ModuleData *mod)
{
	Safety_checkCellVoltageFault(batt, mod);
	Safety_checkCellTemperatureFault(batt, mod);
}

/* ===== Cell Voltage Evaluation ==============================================
 * Cell_Voltage_Fault():
 *  - Scans all cells to determine highest and lowest voltages.
 *  - Applies WARN thresholds for OV/UV.
 *  - Applies FAULT thresholds with counter-based hysteresis; asserts a latched
 *    fault and calls SendFaultSignal() once the counter exceeds its threshold.
 *  - Clears/relaxes a latched fault only after the measurement crosses back by
 *    the configured margin (FAULT_LOCK_MARGIN_*), then calls ClearFaultSignal().
 */
void Safety_checkCellVoltageFault(AccumulatorData *acc, ModuleData *mod)
{
	uint32_t current_time = HAL_GetTick();

	for (int m = 0; m < NUM_MOD; m++)
	{
		Safety_checkPEC(m, &GlobalFaults[m][0], mod, current_time);

		if (!mod[m].pecError)
		{
			for (int c = 0; c < NUM_CELL_PER_MOD; c++)
			{
				FaultFlags_t   *faults           = &GlobalFaults[m][c];
				WarningFlags_t *warns            = &GlobalWarnings[m][c];
				uint16_t        voltage          = mod[m].cellVoltage_mV[c];
				uint16_t        redundantVoltage = mod[m].redundantCellVoltage_mV[c];

				// Safety_checkOpenWire(m, c, voltage, faults, current_time);
				// Safety_checkRedundantVoltage(m, c, voltage, faults, redundantVoltage, current_time);
				Safety_checkOverVoltage(m, c, voltage, faults, warns, current_time);
				Safety_checkUnderVoltage(m, c, voltage, faults, warns, current_time);
			}
		}
	}
}

/* ===== Cell Temperature Evaluation ==========================================
 * Cell_Temperature_Fault():
 *  - Scans all thermistor readings to find highest/lowest temperatures.
 *  - Sets warning when above CELL_HIGH_TEMP_WARNING but below FAULT threshold.
 *  - Uses a small hysteresis counter (e.g., >2) to assert a latched high-temp
 *    fault and SendFaultSignal(); clears once temperature falls below the
 *    threshold by FAULT_LOCK_MARGIN_HIGH_TEMP and calls ClearFaultSignal().
 */
void Safety_checkCellTemperatureFault(AccumulatorData *batt, ModuleData *mod)
{
	uint32_t current_time   = HAL_GetTick();
	for (int m = 0; m < NUM_MOD; m++)
	{
		for (int t = 0; t < NUM_THERM_PER_MOD; t++)
		{
			uint16_t temp = mod[m].pointTemp_C[t];

			FaultFlags_t   *faults = &GlobalFaults[m][t];
			WarningFlags_t *warns  = &GlobalWarnings[m][t];

			Safety_checkOverTemp(m, t, temp, faults, warns, current_time);
			Safety_checkUnderTemp(m, t, temp, faults, warns, current_time);
		}
	}
}

bool Safety_getNextFault(FaultMessage_t *msg)
{
	static int iterator = 0;

	for (int i = 0; i < NUM_CELLS; i++)
	{
		int m = iterator / NUM_CELL_PER_MOD;
		int c = iterator % NUM_CELL_PER_MOD;

		iterator++;
		if (iterator >= NUM_CELLS)
			iterator = 0;

		FaultFlags_t *f = &GlobalFaults[m][c];
		msg->FaultType  = FAULT_NONE;

		if (f->OverVolt)
			msg->FaultType = FAULT_OVER_VOLT;
		else if (f->UnderVolt)
			msg->FaultType = FAULT_UNDER_VOLT;
		else if (f->OverTemp)
			msg->FaultType = FAULT_OVER_TEMP;
		else if (f->UnderTemp)
			msg->FaultType = FAULT_UNDER_TEMP;
		else if (f->RedundancyVolt)
			msg->FaultType = FAULT_REDUNDANT_VOLT;
		else if (f->RedundancyTemp)
			msg->FaultType = FAULT_REDUNDANT_TEMP;
		else if (f->OpenWire)
			msg->FaultType = FAULT_OPEN_WIRE;
		else if (f->PEC)
			msg->FaultType = FAULT_PEC;

		if (msg->FaultType != FAULT_NONE)
		{
			msg->ModuleID = m;
			msg->CellID   = c;
			return true;
		}
	}

	return false;
}

bool Safety_getNextWarning(WarningMessage_t *msg)
{
	static int iterator = 0;

	for (int i = 0; i < NUM_CELLS; i++)
	{
		int m = iterator / NUM_CELL_PER_MOD;
		int c = iterator % NUM_CELL_PER_MOD;

		iterator++;
		if (iterator >= NUM_CELLS)
			iterator = 0;

		WarningFlags_t *w = &GlobalWarnings[m][c];
		msg->WarningType = WARNING_NONE;

		if (w->OverVoltWarn)
			msg->WarningType = WARNING_OVER_VOLT;
		else if (w->UnderVoltWarn)
			msg->WarningType = WARNING_UNDER_VOLT;
		else if (w->OverTempWarn)
			msg->WarningType = WARNING_OVER_TEMP;
		else if (w->UnderTempWarn)
			msg->WarningType = WARNING_UNDER_TEMP;
		else if (w->ImbalanceWarn)
			msg->WarningType = WARNING_IMBALANCE;

		if (msg->WarningType != WARNING_NONE)
		{
			msg->ModuleID = m;
			msg->CellID   = c;
			return true;
		}
	}

	return false;
}

void Safety_getModuleFaultBits(uint16_t *faultBuffer)
{
	for (uint8_t m = 0; m < NUM_MOD; m++)
    {
        uint16_t faultBits = 0;
        
        for (uint8_t c = 0; c < NUM_CELL_PER_MOD; c++)
        {
            FaultFlags_t *f = &GlobalFaults[m][c];

            if (f->UnderVolt || f->OpenWire || f->PEC || f->OverTemp ||
                f->UnderTemp || f->OverVolt || f->RedundancyVolt || f->RedundancyTemp)
            {
                faultBits |= (uint16_t)(1u << c);
            }
        }
        
        faultBuffer[m] = faultBits;
    }
}

static void Safety_clearFaults(void)
{
	for (int m = 0; m < NUM_MOD; m++)
	{
		for (int c = 0; c < NUM_CELL_PER_MOD; c++)
		{
			FaultFlags_t *f = &GlobalFaults[m][c];

			if (f->OverVolt || f->UnderVolt || f->OverTemp || f->UnderTemp || f->OpenWire || f->PEC ||
				f->RedundancyVolt || f->RedundancyTemp)
			{
				return;
			}
		}
	}

	ClearFaultSignal();
}

static void Safety_checkPEC(int m, FaultFlags_t *module_faults, ModuleData *mod, uint32_t current_time)
{
	if (mod[m].pecError)
	{
		if (Timer_PEC[m] == 0)
		{
			Timer_PEC[m] = current_time;
		}
		else if ((current_time - Timer_PEC[m]) > TIME_LIMIT_PEC && module_faults[0].PEC == 0)
		{
			for (int c = 0; c < NUM_CELL_PER_MOD; c++)
			{
				FaultFlags_t *faults = &module_faults[c];
				faults->PEC          = 1;
			}

			SendFaultSignal();
		}
	}
	else
	{
		Timer_PEC[m] = 0;

		if (module_faults[0].PEC == 1)
		{
			for (int c = 0; c < NUM_CELL_PER_MOD; c++)
			{
				FaultFlags_t *faults = &module_faults[c];
				faults->PEC          = 0;
			}

			Safety_clearFaults();
		}
	}
}

static void Safety_checkOpenWire(int m, int c, uint16_t voltage, FaultFlags_t *faults, uint32_t current_time)
{
	if (voltage < CELL_OPEN_WIRE_FAULT)
	{
		if (Timer_OpenWire[m][c] == 0)
		{
			Timer_OpenWire[m][c] = current_time;
		}
		else if ((current_time - Timer_OpenWire[m][c] > TIME_LIMIT_OPEN_WIRE) && faults->OpenWire == 0)
		{
			faults->OpenWire = 1;
			SendFaultSignal();
		}
	}
	else
	{
		Timer_OpenWire[m][c] = 0;

		if (faults->OpenWire == 1)
		{
			faults->OpenWire = 0;
			Safety_clearFaults();
		}
	}
}

static void Safety_checkRedundantVoltage(int m, int c, uint16_t voltage, FaultFlags_t *faults,
										 uint16_t redundantVoltage, uint32_t current_time)
{
	int32_t diff = (int32_t)voltage - (int32_t)redundantVoltage;

	if (diff < 0)
	{
		diff = -diff;
	}

	if (diff > REDUNDANCY_VOLT_FAULT)
	{
		if (Timer_RedundancyVolt[m][c] == 0)
		{
			Timer_RedundancyVolt[m][c] = current_time;
		}
		else if ((current_time - Timer_RedundancyVolt[m][c]) > TIME_LIMIT_REDUNDANCY_VOLT &&
				 faults->RedundancyVolt == 0)
		{
			faults->RedundancyVolt = 1;

			/*if (m == 0 && c ==0)
			{
				printf("voltage %d\r redundantVoltage %d\r module %d \r cell %d \r faults %d \r diff %d \r\n",
			mod[m].cell_volt[c], mod[m].redundantCellVoltage_mV[c], m, c, *(uint8_t*)&GlobalFaults[m][c], diff);
			}
			*/

			SendFaultSignal();
		}
	}
	else
	{
		Timer_RedundancyVolt[m][c] = 0;

		if (diff < (REDUNDANCY_VOLT_FAULT - FAULT_LOCK_MARGIN_REDUNDANCY_VOLT) && faults->RedundancyVolt == 1)
		{
			faults->RedundancyVolt = 0;
			Safety_clearFaults();
		}
	}
}

static void Safety_checkOverVoltage(int m, int c, int16_t voltage, FaultFlags_t *faults, WarningFlags_t *warns,
									uint32_t current_time)
{
	if (voltage >= CELL_HIGH_VOLT_WARNING)
	{
		warns->OverVoltWarn = 1;
	}
	else
	{
		warns->OverVoltWarn = 0;
	}

	if (voltage >= CELL_HIGH_VOLT_FAULT)
	{
		if (Timer_OverVolt[m][c] == 0)
		{
			Timer_OverVolt[m][c] = current_time;
		}

		else if ((current_time - Timer_OverVolt[m][c]) > TIME_LIMIT_OVER_VOLT && faults->OverVolt == 0)
		{
			faults->OverVolt = 1;
			SendFaultSignal();
		}
	}
	else
	{
		Timer_OverVolt[m][c] = 0;

		if (voltage < (CELL_HIGH_VOLT_FAULT - FAULT_LOCK_MARGIN_HIGH_VOLT) && faults->OverVolt == 1)
		{
			faults->OverVolt = 0;
			Safety_clearFaults();
		}
	}
}

static void Safety_checkUnderVoltage(int m, int c, int16_t voltage, FaultFlags_t *faults, WarningFlags_t *warns,
									 uint32_t current_time)
{
	if (voltage <= CELL_LOW_VOLT_WARNING)
	{
		warns->UnderVoltWarn = 1;
	}
	else
	{
		warns->UnderVoltWarn = 0;
	}

	if (voltage <= CELL_LOW_VOLT_FAULT)
	{
		if (Timer_UnderVolt[m][c] == 0)
		{
			Timer_UnderVolt[m][c] = current_time;
		}
		else if ((current_time - Timer_UnderVolt[m][c]) > TIME_LIMIT_UNDER_VOLT && faults->UnderVolt == 0)
		{
			faults->UnderVolt = 1;
			SendFaultSignal();
		}
	}
	else
	{
		Timer_UnderVolt[m][c] = 0;

		if (voltage > (CELL_LOW_VOLT_FAULT + FAULT_LOCK_MARGIN_LOW_VOLT) && faults->UnderVolt == 1)
		{
			faults->UnderVolt = 0;
			Safety_clearFaults();
		}
	}
}

static void Safety_checkOverTemp(int m, int t, uint16_t temp, FaultFlags_t *faults, WarningFlags_t *warns,
								 uint32_t current_time)
{
	if (temp >= CELL_HIGH_TEMP_WARNING)
	{
		warns->OverTempWarn = 1;
	}
	else
	{
		warns->OverTempWarn = 0;
	}

	if (temp >= CELL_HIGH_TEMP_FAULT)
	{
		if (Timer_OverTemp[m][t] == 0)
		{
			Timer_OverTemp[m][t] = current_time;
		}
		else if ((current_time - Timer_OverTemp[m][t]) > TIME_LIMIT_OVER_TEMP && faults->OverTemp == 0)
		{
			faults->OverTemp = 1;
			SendFaultSignal();
		}
	}
	else
	{
		Timer_OverTemp[m][t] = 0;

		if (temp < (CELL_HIGH_TEMP_FAULT - FAULT_LOCK_MARGIN_TEMP) && faults->OverTemp == 1)
		{
			faults->OverTemp = 0;
			Safety_clearFaults();
		}
	}
}

static void Safety_checkUnderTemp(int m, int t, uint16_t temp, FaultFlags_t *faults, WarningFlags_t *warns,
								  uint32_t current_time)
{
	if (temp <= CELL_LOW_TEMP_WARNING)
	{
		warns->UnderTempWarn = 1;
	}
	else
	{
		warns->UnderTempWarn = 0;
	}

	if (temp <= CELL_LOW_TEMP_FAULT)
	{
		if (Timer_UnderTemp[m][t] == 0)
		{
			Timer_UnderTemp[m][t] = current_time;
		}
		else if ((current_time - Timer_UnderTemp[m][t]) > TIME_LIMIT_UNDER_TEMP && faults->UnderTemp == 0)
		{
			faults->UnderTemp = 1;
			SendFaultSignal();
		}
	}
	else
	{
		Timer_UnderTemp[m][t] = 0;

		if (temp > (CELL_LOW_TEMP_FAULT + FAULT_LOCK_MARGIN_TEMP) && faults->UnderTemp == 1)
		{
			faults->UnderTemp = 0;
			Safety_clearFaults();
		}
	}
}

