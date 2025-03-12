#include <Arduino.h>

// Battery functions
/** Definition of the Analog input that is connected to the battery voltage divider */
#define PIN_VBAT A0
/** Definition of milliVolt per LSB => 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096 */
#define VBAT_MV_PER_LSB (0.73242188F)
/** Voltage divider value => 1.5M + 1M voltage divider on VBAT = (1.5M / (1M + 1.5M)) */
#define VBAT_DIVIDER (0.4F)
/** Compensation factor for the VBAT divider */
#define VBAT_DIVIDER_COMP (1.73)
/** Fixed calculation of milliVolt from compensation value */
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
void initReadVBAT(void);
uint8_t readBatt(void);
uint8_t lorawanBattLevel(void);
float readVBAT(void);
extern uint8_t battLevel;