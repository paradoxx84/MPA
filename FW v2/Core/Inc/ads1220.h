#ifndef ADS_1220_H
#define ADS_1220_H

#include "stdint.h"
#include "spi.h"

typedef enum
{       
    stADCDisable,
    stADCResetting,    
    stADCMainConfig,
    //stADCConfigureForInternalTemperature,
    stADCConfigureForCalibration,
    stADCConfigureForCH0,    
    //stADCTemperature,
    stADCCalibration,
    stADC_CH0
}ADS1220_STATE_t;

typedef struct
{
    ADS1220_STATE_t ADC_STATE;
    uint8_t REGISTERS[4];
    uint8_t NewDataAvailable;
    uint64_t LastUpdate;
    int32_t Ch0;
    int32_t Offset;
    //float CurrentTemperature;
    //float CalibratedTemperature;
    //uint64_t NextTemperatureCheck;
    uint64_t NextCalibration;
    uint8_t TXbuf[3];
    uint8_t RXbuf[3];
    //uint8_t DMAbuf[3];
    uint16_t CalibCounter;
    int32_t CalibSum;
    SPI_HandleTypeDef* InterfaceHandle;
    GPIO_TypeDef* CSPort;
    uint16_t CSPin;
    GPIO_TypeDef* RDYPort;
    uint16_t RDYPin;    
}ADC_ADS1220;

#define ADS1220_DUMMY       0x00

#define ADS1220_CMD_RESET   0x06
#define ADS1220_CMD_START   0x08
#define ADS1220_CMD_PWRDN   0x02
#define ADS1220_CMD_RDATA   0x10
#define ADS1220_CMD_RREG    0x20
#define ADS1220_CMD_WREG    0x40

#define ADS1220_MUX_REG     0x00
#define ADS1220_MUX_REG_I   0x00
#define ADS1220_MUX_MASK    0xF0
#define ADS1220_MUX_01      0x00
#define ADS1220_MUX_2SINGLE 0xA0
#define ADS1220_MUX_ZCAL    0xE0

#define ADS1220_GAIN_REG    0x00
#define ADS1220_GAIN_REG_I  0x00
#define ADS1220_GAIN_MASK   0x0E
#define ADS1220_GAIN_1      0x00
#define ADS1220_GAIN_2      0x02
#define ADS1220_GAIN_4      0x04
#define ADS1220_GAIN_8      0x06
#define ADS1220_GAIN_16     0x08
#define ADS1220_GAIN_32     0x0A
#define ADS1220_GAIN_64     0x0C
#define ADS1220_GAIN_128    0x0E

#define ADS1220_DR_REG      0x04
#define ADS1220_DR_REG_I    0x01
#define ADS1220_DR_MASK     0xE0
#define ADS1220_DR_20       0x00
#define ADS1220_DR_45       0x20
#define ADS1220_DR_90       0x40
#define ADS1220_DR_175      0x60
#define ADS1220_DR_330      0x80
#define ADS1220_DR_600      0xA0
#define ADS1220_DR_1000     0xC0

#define ADS1220_MODE_REG    0x04
#define ADS1220_MODE_REG_I  0x01
#define ADS1220_MODE_MASK   0x18
#define ADS1220_MODE_NORM   0x00
#define ADS1220_MODE_DUTY   0x08
#define ADS1220_MODE_TURB   0x10

#define ADS1220_CM_REG      0x04
#define ADS1220_CM_REG_I    0x01
#define ADS1220_CM_MASK     0x04
#define ADS1220_CM_SINGLE   0x00
#define ADS1220_CM_CONT     0x04

#define ADS1220_TS_REG      0x04
#define ADS1220_TS_REG_I    0x01
#define ADS1220_TS_MASK     0x02
#define ADS1220_TS_OFF      0x00
#define ADS1220_TS_ON       0x02

#define ADS1220_VREF_REG    0x08
#define ADS1220_VREF_REG_I  0x02
#define ADS1220_VREF_MASK   0xC0
#define ADS1220_VREF_INT    0x00
#define ADS1220_VREF_REF0   0x40
#define ADS1220_VREF_REF1   0x80
#define ADS1220_VREF_AVDD   0xC0

#define ADS1220_FIR_REG     0x08
#define ADS1220_FIR_REG_I   0x02
#define ADS1220_FIR_MASK    0x30
#define ADS1220_FIR_OFF     0x00
#define ADS1220_FIR_5060    0x10
#define ADS1220_FIR_50      0x20
#define ADS1220_FIR_60      0x30

#define ADS1220_PSW_REG     0x08
#define ADS1220_PSW_REG_I   0x02
#define ADS1220_PSW_MASK    0x08
#define ADS1220_PSW_OFF     0x00
#define ADS1220_PSW_AUTO    0x08

#define ADS1220_IDAC_REG    0x08
#define ADS1220_IDAC_REG_I  0x02
#define ADS1220_IDAC_MASK   0x07
#define ADS1220_IDAC_OFF    0x00
#define ADS1220_IDAC_10     0x01
#define ADS1220_IDAC_50     0x02
#define ADS1220_IDAC_100    0x03
#define ADS1220_IDAC_250    0x04
#define ADS1220_IDAC_500    0x05
#define ADS1220_IDAC_1000   0x06
#define ADS1220_IDAC_1500   0x07

#define ADS1220_IDAC1_REG   0x0C
#define ADS1220_IDAC1_REG_I 0x03
#define ADS1220_IDAC2_REG   0x0C
#define ADS1220_IDAC2_REG_I 0x03

#define ADS1220_IDAC1_MASK  0xE0
#define ADS1220_IDAC2_MASK  0x1C

#define ADS1220_IDAC1_OFF   0x00
#define ADS1220_IDAC1_AIN0  0x20
#define ADS1220_IDAC1_AIN1  0x40
#define ADS1220_IDAC1_AIN2  0x60
#define ADS1220_IDAC1_AIN3  0x80
#define ADS1220_IDAC1_REFP0 0xA0
#define ADS1220_IDAC1_REFN0 0xC0

#define ADS1220_IDAC2_OFF   0x00
#define ADS1220_IDAC2_AIN0  0x04
#define ADS1220_IDAC2_AIN1  0x08
#define ADS1220_IDAC2_AIN2  0x0C
#define ADS1220_IDAC2_AIN3  0x10
#define ADS1220_IDAC2_REFP0 0x14
#define ADS1220_IDAC2_REFN0 0x18

typedef enum
{
    gain_1 = ADS1220_GAIN_1,
    gain_2 = ADS1220_GAIN_2,
    gain_4 = ADS1220_GAIN_4,
    gain_8 = ADS1220_GAIN_8,
    gain_16 = ADS1220_GAIN_16,
    gain_32 = ADS1220_GAIN_32,
    gain_64 = ADS1220_GAIN_64,
    gain_128 = ADS1220_GAIN_128
}ads1220_gain_t;

uint8_t ByteToGain(uint8_t gain)
{
    uint8_t result = 0;
    switch (gain)
    {
        case 0:
        {
            result = ADS1220_GAIN_1;
            break;
        }
        case 1:
        {
            result = ADS1220_GAIN_2;
            break;
        }
        case 2:
        {
            result = ADS1220_GAIN_4;
            break;
        }
        case 3:
        {
            result = ADS1220_GAIN_8;
            break;
        }
        case 4:
        {
            result = ADS1220_GAIN_16;
            break;
        }
        case 5:
        {
            result = ADS1220_GAIN_32;
            break;
        }
        case 6:
        {
            result = ADS1220_GAIN_64;
            break;
        }
        case 7:
        {
            result = ADS1220_GAIN_128;
            break;
        }            
    }
    return result;
}

uint8_t GainToByte(uint8_t arg)
{
    uint8_t result = 0;
    switch (arg)
    {
        case ADS1220_GAIN_1:
        {
            result = 0;
            break;
        }
        case ADS1220_GAIN_2:
        {
            result = 1;
            break;
        }
        case ADS1220_GAIN_4:
        {
            result = 2;
            break;
        }
        case ADS1220_GAIN_8:
        {
            result = 3;
            break;
        }
        case ADS1220_GAIN_16:
        {
            result = 4;
            break;
        }
        case ADS1220_GAIN_32:
        {
            result = 5;
            break;
        }
        case ADS1220_GAIN_64:
        {
            result = 6;
            break;
        }
        case ADS1220_GAIN_128:
        {
            result = 7;
            break;
        }            
    }
    return result;
}

#endif
