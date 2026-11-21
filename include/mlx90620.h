#ifndef MLX90620_H
#define MLX90620_H

#include <soft_i2c.h>

#ifdef __cplusplus
extern "C" {
#endif
//Begin registers
//refreshRate Possible values: Hz_LSB_0=0.5Hz, Hz_LSB_1=1Hz, Hz_LSB_2=2Hz, Hz_LSB_4=4Hz, Hz_LSB_8=8Hz, Hz_LSB_16=16Hz, Hz_LSB_32=32Hz.
#define Hz_LSB_0 0x3F
#define Hz_LSB_1 0x3E
#define Hz_LSB_2 0x3D
#define Hz_LSB_4 0x3C
#define Hz_LSB_8 0x3B
#define Hz_LSB_16 0x3A
#define Hz_LSB_32 0x39
#define Hz_LSB_default 0x3E

#define CAL_ACOMMON_L 0xD0
#define CAL_ACOMMON_H 0xD1
#define CAL_ACP_L 0xD3
#define CAL_ACP_H 0xD4
#define CAL_BCP 0xD5
#define CAL_alphaCP_L 0xD6
#define CAL_alphaCP_H 0xD7
#define CAL_TGC 0xD8
#define CAL_AI_SCALE 0xD9
#define CAL_BI_SCALE 0xD9

#define VTH_L 0xDA
#define VTH_H 0xDB
#define KT1_L 0xDC
#define KT1_H 0xDD
#define KT2_L 0xDE
#define KT2_H 0xDF
#define KT_SCALE 0xD2

//Common sensitivity coefficients
#define CAL_A0_L 0xE0
#define CAL_A0_H 0xE1
#define CAL_A0_SCALE 0xE2
#define CAL_DELTA_A_SCALE 0xE3
#define CAL_EMIS_L 0xE4
#define CAL_EMIS_H 0xE5

//Config register = 0xF5-F6
#define OSC_TRIM_VALUE 0xF7

enum { MLX90620_OK = 0, MLX90620_ERROR = 1 };

typedef struct {
	byte eeprom[256];
	int16_t cpix;
	uint16_t ptat;
	int16_t resolution;
	int16_t configuration;
	int16_t irData[64];
	float temperatures[64];
	float tambient;
	uint8_t check; /* MLX90620_OK or MLX90620_ERROR after sensor operations. */
} MLX90620;

void initSensor(MLX90620*);
void readEEPROM(MLX90620*);
void writeTrimmingValue(MLX90620*);
void setConfiguration(void);
bool readConfigChecked(uint16_t *config);
int16_t readConfig(void);
bool checkConfig(void);
void readRes(MLX90620*);
void readPTAT(MLX90620*);
void calculateTA(MLX90620*);
void readCPIX(MLX90620*);
void readIR(MLX90620*);
void calculateTO(MLX90620*);
#ifdef __cplusplus
}
#endif
#endif
