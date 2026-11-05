/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
#include "ads1220.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PASCAL_SIZE 500
#define DEFAULT_VAC 13.6515045f
typedef enum
{
    unit_mbar = 0,
    unit_Torr = 1,
    unit_Pa = 2
}unit_t;

typedef struct
{
    uint8_t OK;
    uint8_t Status;
    uint8_t Ready;
    float Pressure;
    float PascalPressure;
    float PascalPressureFiltered;
    uint16_t Raw;
    float FactorA;
    float FactorB;
    float FSR_Mantissa;
    float FSR_Exp;
    unit_t Unit;
    uint64_t TimeStamp;
    float Pascals[PASCAL_SIZE];
    float PascalSum;
    uint16_t PascalIdx;
    uint32_t Errors;    
}inficon_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FWREV 1.01f

#define INFICON_UART huart1
#define PC_UART huart3
#define I2C420 hi2c2
#define LASER_UART huart6

#define LED_ON 30
#define LED_TICK 300

#define SOF 0x3B
#define PC_RX_BUF_SIZE 256
#define COMMAND_LENGTH 14

#define RPM_MIN 20
#define RPM_MAX 80

#define PROBE_420 100.0f
#define REF_420 2.048f
#define INTERVAL_420 150

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint64_t uptime = 0;
uint64_t LedNextUpdate = 500;
uint64_t MotorNextProcess = 1000;
uint64_t Next420Check = 500;

int16_t StatusRegister = 1;

uint8_t LedState = 0; //0 - blinking OFF, 1 - blinking ON, 2 - constant ON

uint8_t PowerOffAllowed = 0;

uint8_t PC_RX_BUF[PC_RX_BUF_SIZE];
uint8_t command_buf[COMMAND_LENGTH]; //for parser
int8_t PC_RX_flag = 0;
uint64_t LastPacketReceived;
uint32_t PC_TimeOut = 10;
uint32_t ErrCounter = 0;

//inficon
uint8_t INFICONRXBUF[50];
volatile uint8_t InficonRX = 0;
inficon_t I;
uint64_t InficonLastGoodPacket = 0;

//4-20
uint8_t DataReady420 = 0;
float I420 = 0;
uint8_t ADS1110addr = 0x48;          
uint8_t ADS1110buffer[2];
uint8_t Status420 = 0;
uint64_t ADS1110LastGoodPacket = 0;

//motor
uint8_t RunButtonState = 2;
uint64_t LastRunButtonStateChanged;
uint8_t ButtonStateChanged = 0;
int8_t DefaultSpeedSetPoint = 30; //rpm, default 30
int8_t SpeedSetPoint = 0; 
int8_t NewSpeedSetPoint = 0;
uint8_t DefaultAccTime = 10; //default 10
uint8_t AccTime = 10;      
float RawCurrentSpeed = 0;
float PerMillisecond = 0;

//vent
uint16_t Venting = 0;
uint64_t VentNextSecond;
uint8_t VentBtn;
//inputs
uint8_t Input1, Input2, Input3, Input4;

//pt1000
ADC_ADS1220 hadc;
uint64_t Delay;
uint8_t RDYPin = 1, PrevRDYPin = 1;
float Pt1000Temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint64_t GetUpTime(void);
void ParseInficon(uint8_t*, inficon_t*, uint8_t full);
void CommandExecute(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  I.OK = 0;
  I.TimeStamp = 0;
  I.PascalSum = DEFAULT_VAC * PASCAL_SIZE;
  for(uint16_t i = 0; i < PASCAL_SIZE; i++)
  {
      I.Pascals[i] = DEFAULT_VAC;
  }  
  I.PascalIdx = 0;
  I.Errors = 0;
  HAL_UARTEx_ReceiveToIdle_IT(&INFICON_UART, INFICONRXBUF, sizeof INFICONRXBUF);
  
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  
    
//  
    hadc.InterfaceHandle = &hspi1;
    hadc.CSPort = CSPT_GPIO_Port;
    hadc.CSPin = CSPT_Pin;
    hadc.RDYPort = RDYPT_GPIO_Port;
    hadc.RDYPin = RDYPT_Pin;    
    hadc.NextCalibration = 2000;
    hadc.ADC_STATE = stADCDisable;
  
  while (1)
  {
      if(Venting)
      {
          HAL_GPIO_WritePin(VENTSIG_GPIO_Port, VENTSIG_Pin, GPIO_PIN_RESET);
          if(GetUpTime() >= VentNextSecond)
          {
              VentNextSecond = GetUpTime() + 1000;
              Venting--;
          }
      }else{
          HAL_GPIO_WritePin(VENTSIG_GPIO_Port, VENTSIG_Pin, GPIO_PIN_SET);
      }
      
// MOTOR      
      uint8_t NewRunButtonState = HAL_GPIO_ReadPin(AUTO_BTN_GPIO_Port, AUTO_BTN_Pin);
      if(NewRunButtonState != RunButtonState)
      {
          LastRunButtonStateChanged = GetUpTime();
          ButtonStateChanged = 1;
      }else{
          if(GetUpTime() > LastRunButtonStateChanged + 200)
          {
              if(ButtonStateChanged)
              {
                  ButtonStateChanged = 0;
                  if(RunButtonState)
                  {
                      NewSpeedSetPoint = DefaultSpeedSetPoint;
                      AccTime = DefaultAccTime;
                  }else{
                  }
              }
          }
      }
      RunButtonState = NewRunButtonState;
      if(GetUpTime() >= MotorNextProcess)
      {
          MotorNextProcess += 4;          
          
          if(NewSpeedSetPoint != SpeedSetPoint)
          {
              SpeedSetPoint = NewSpeedSetPoint;              
              PerMillisecond = 4.0f * (SpeedSetPoint - RawCurrentSpeed) / (AccTime * 1000);
          }
          
          RawCurrentSpeed += PerMillisecond;
          if(PerMillisecond > 0)
          {
              if(RawCurrentSpeed > SpeedSetPoint)
              {
                  RawCurrentSpeed = SpeedSetPoint;
                  PerMillisecond = 0;
              }
          }else{
              if(RawCurrentSpeed < SpeedSetPoint)
              {
                  RawCurrentSpeed = SpeedSetPoint;
                  PerMillisecond = 0;
              }
          }
          if(fabsf(RawCurrentSpeed) < 0.2f)              
          {
              HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
          }else{
              HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
          }
          uint32_t arr = 84000000. * 60 / (2 * fabsf(RawCurrentSpeed)  * 6400 * 1.9f) - 1;
          if(arr > 2072026)
          {
              arr = 2072026;
          }
          __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
      }
      
      if((SpeedSetPoint != 0) && (fabsf(RawCurrentSpeed - SpeedSetPoint) < 1))
      {
          HAL_GPIO_WritePin(MOTORLED_GPIO_Port, MOTORLED_Pin, GPIO_PIN_SET);
      }else{
          HAL_GPIO_WritePin(MOTORLED_GPIO_Port, MOTORLED_Pin, GPIO_PIN_RESET);
      }
// end MOTOR          
      
      if(HAL_GPIO_ReadPin(POSITION0_GPIO_Port, POSITION0_Pin))
      {
          Input1 = 0;
      }else{
          Input1 = 1;
      }
      if(HAL_GPIO_ReadPin(POSITION1_GPIO_Port, POSITION1_Pin))
      {
          Input2 = 0;
      }else{
          Input2 = 1;
      }
      if(HAL_GPIO_ReadPin(POSITION2_GPIO_Port, POSITION2_Pin))
      {
          Input3 = 0;
      }else{
          Input3 = 1;
      }
      if(HAL_GPIO_ReadPin(POSITION3_GPIO_Port, POSITION3_Pin))
      {
          Input4 = 0;
      }else{
          Input4 = 1;
      }
      
      if(!HAL_GPIO_ReadPin(PWRBTN_GPIO_Port, PWRBTN_Pin))
      {
          StatusRegister |= 0x02;
      }
      if(HAL_GPIO_ReadPin(VENT_BTN_GPIO_Port, VENT_BTN_Pin))
      {
          StatusRegister |= 0x04;
      }
      if(RunButtonState)
      {
          StatusRegister |= 0x08;
      }else{
          StatusRegister &= ~0x08;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      
      if(GetUpTime() >= Next420Check)
      {   
          if(GetUpTime() >= ADS1110LastGoodPacket + 2000)
          {
              Status420 = 0;
              Next420Check = GetUpTime() + 1000;
              uint8_t GeneralCallAddress = 0;
              uint8_t ResetCmd = 0x06;
              HAL_I2C_Master_Transmit(&hi2c1, GeneralCallAddress, &ResetCmd, 1, 20);
          }else{
              Next420Check = GetUpTime() + INTERVAL_420;
              HAL_I2C_Master_Receive_IT(&hi2c2, ADS1110addr << 1, ADS1110buffer, 2);
          }
      }
      if(DataReady420)
      {          
          DataReady420 = 0;
          Status420 = 1;
          int16_t raw_value = (int16_t)((ADS1110buffer[0] << 8) | ADS1110buffer[1]);
          int16_t fs = 0x7fff;
          float volt = (REF_420 / fs) * raw_value;
          I420 = 1000 * volt / PROBE_420; //mA
          ADS1110LastGoodPacket = GetUpTime();          
      }
      
            
      if((InficonRX == 9) && (INFICONRXBUF[0] == 7))
      {
          InficonRX = 0;
          uint8_t CheckSum = 0;
          for(uint8_t i = 1; i <= 7; i++)
          {
              CheckSum += INFICONRXBUF[i];
          }          
          if(CheckSum == INFICONRXBUF[8])
          {
              I.OK = 1;
              I.TimeStamp = GetUpTime();
              ParseInficon(INFICONRXBUF, &I, 1);
          }else{              
              I.Errors++;
          }
      }else{
          InficonRX = 0;
          if((I.Errors > 500) || (GetUpTime() > I.TimeStamp + 2000))
          {
              I.OK = 0;
          }
      }
      
      if(GetUpTime() >= LedNextUpdate)
      {
          LedNextUpdate += LED_TICK;
                                  
          switch(LedState)
          {
              case 0: //blinking, OFF
              {   
                  HAL_GPIO_WritePin(PWRLED_GPIO_Port, PWRLED_Pin, GPIO_PIN_SET);
                  LedState = 1;
                  break;
              }
              case 1: //blinking, ON
              {
                  HAL_GPIO_WritePin(PWRLED_GPIO_Port, PWRLED_Pin, GPIO_PIN_RESET);                  
                  LedState = 0;
                  break;
              }                 
              case 2: //set constant ON
              {                  
                  HAL_GPIO_WritePin(PWRLED_GPIO_Port, PWRLED_Pin, GPIO_PIN_SET);
                  LedState = 3;                  
                  break;
              }
              case 3: // constant ON
              {
                  break;
              }
          }
      }
      
      if(PC_TimeOut > 10)
      {            
          if(LedState == 3)
          {            
              LedState = 0;
          }
      }
    
      uint64_t tempTick = GetUpTime();
      if(PC_RX_flag)
      {
          if((PC_RX_flag == COMMAND_LENGTH) && (PC_RX_BUF[0] == SOF))
          {
              LastPacketReceived = tempTick;
              PC_TimeOut = 0;
              LedState = 2;
             
              for(uint8_t i = 0; i < PC_RX_flag; i++)
              {                    
                  command_buf[i] = PC_RX_BUF[i];
              }
              CommandExecute();
          }else{
              ErrCounter++;
          }
          HAL_UART_AbortReceive_IT(&PC_UART);
          while(HAL_UARTEx_ReceiveToIdle_IT(&PC_UART, PC_RX_BUF, PC_RX_BUF_SIZE) != HAL_OK);
          __HAL_UART_DISABLE_IT(&PC_UART, UART_IT_ERR);
          PC_RX_flag = 0;            
      }else{
          if(tempTick > LastPacketReceived + 200)
          {
              LastPacketReceived = tempTick;
              PC_TimeOut ++;
              HAL_UART_AbortReceive_IT(&PC_UART);            
              while(HAL_UARTEx_ReceiveToIdle_IT(&PC_UART, PC_RX_BUF, PC_RX_BUF_SIZE) != HAL_OK);
              __HAL_UART_DISABLE_IT(&PC_UART, UART_IT_ERR);
            
              if(PowerOffAllowed)
              {                
                  if(PC_TimeOut > 100)
                  {                      
                      HAL_GPIO_WritePin(KILL_GPIO_Port, KILL_Pin, GPIO_PIN_SET);
                  }
              }
          }            
      }
      
      switch(hadc.ADC_STATE)
      {
          case stADCDisable:
          {
              
              hadc.TXbuf[0] = ADS1220_CMD_RESET;
              HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
              while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 1, 2) == HAL_BUSY);
              HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
              Delay = GetUpTime() + 2;            
              hadc.ADC_STATE = stADCResetting;     
              break;
          }
          case stADCResetting:
          {              
              if(GetUpTime() >= Delay)
              {
                  hadc.REGISTERS[0] = 0x00; 
                  hadc.REGISTERS[1] = 0x00;
                  hadc.REGISTERS[2] = 0x00; // По даташиту дефолт 0x00 (ток выкл)
                  hadc.REGISTERS[3] = 0x00; // По даташиту дефолт 0x00 (выводы выкл)
                  hadc.ADC_STATE = stADCMainConfig;
                  //hadc[adc].NextTemperatureCheck = GetUpTime()+ ADC_INTERNAL_TEMP_CHECK_INTERVAL;
                  hadc.NewDataAvailable = 0;
              }
              break;
          }
            case stADCMainConfig: // Настройка глобальных параметров (Регистры 1, 2, 3)
            {                   
                // 1. Скорость 20 SPS и режим Normal (вместо Turbo, чтобы работал FIR фильтр помех)
                hadc.REGISTERS[ADS1220_DR_REG_I] &= ~ADS1220_DR_MASK;
                hadc.REGISTERS[ADS1220_DR_REG_I] |= ADS1220_DR_20; // data rate 20 SPS            
                hadc.REGISTERS[ADS1220_MODE_REG_I] &= ~ADS1220_MODE_MASK;
                hadc.REGISTERS[ADS1220_MODE_REG_I] |= ADS1220_MODE_NORM; // Режим Normal
                
                hadc.TXbuf[0] = ADS1220_CMD_WREG | ADS1220_DR_REG;
                hadc.TXbuf[1] = hadc.REGISTERS[ADS1220_DR_REG_I]; 
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
                while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 2, 2) == HAL_BUSY);
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
                                          
                // 2. Режим непрерывного преобразования
                hadc.REGISTERS[ADS1220_CM_REG_I] &= ~ADS1220_CM_MASK;
                hadc.REGISTERS[ADS1220_CM_REG_I] |= ADS1220_CM_CONT; // continuous mode    
                hadc.TXbuf[0] = ADS1220_CMD_WREG | ADS1220_CM_REG;
                hadc.TXbuf[1] = hadc.REGISTERS[ADS1220_CM_REG_I];
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
                while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 2, 2) == HAL_BUSY);
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);

                // 3. Включение источника тока 250 мкА и фильтра сети 50/60 Гц (Регистр 2)
                hadc.REGISTERS[ADS1220_IDAC_REG_I] &= ~ADS1220_IDAC_MASK;
                hadc.REGISTERS[ADS1220_IDAC_REG_I] |= ADS1220_IDAC_250; // Ток 250 мкА
                hadc.REGISTERS[ADS1220_FIR_REG_I] &= ~ADS1220_FIR_MASK;
                hadc.REGISTERS[ADS1220_FIR_REG_I] |= ADS1220_FIR_5060; // Фильтр 50/60 Гц
                
                hadc.TXbuf[0] = ADS1220_CMD_WREG | ADS1220_IDAC_REG;
                hadc.TXbuf[1] = hadc.REGISTERS[ADS1220_IDAC_REG_I];
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
                while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 2, 2) == HAL_BUSY);
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);

                // 4. Коммутация тока IDAC1 на ножку AIN3 (Регистр 3)
                hadc.REGISTERS[ADS1220_IDAC1_REG_I] &= ~ADS1220_IDAC1_MASK;
                hadc.REGISTERS[ADS1220_IDAC1_REG_I] |= ADS1220_IDAC1_AIN3; // Ток летит в AIN3
                hadc.REGISTERS[ADS1220_IDAC2_REG_I] &= ~ADS1220_IDAC2_MASK;
                hadc.REGISTERS[ADS1220_IDAC2_REG_I] |= ADS1220_IDAC2_OFF;  // Второй источник отключен
                
                hadc.TXbuf[0] = ADS1220_CMD_WREG | ADS1220_IDAC1_REG;
                hadc.TXbuf[1] = hadc.REGISTERS[ADS1220_IDAC1_REG_I];
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
                while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 2, 2) == HAL_BUSY);
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
              
                hadc.ADC_STATE = stADCConfigureForCH0;  
                break;
            }
            case stADCConfigureForCH0: // Настройка измерительного канала (Регистр 0)
            {   
                // 1. Усиление (Gain = 4 для комнатных температур)
                hadc.REGISTERS[ADS1220_GAIN_REG_I] &= ~ADS1220_GAIN_MASK;                                          
                hadc.REGISTERS[ADS1220_GAIN_REG_I] |= ADS1220_GAIN_4; // gain = 4   
                hadc.TXbuf[0] = ADS1220_CMD_WREG | ADS1220_GAIN_REG;
                hadc.TXbuf[1] = hadc.REGISTERS[ADS1220_GAIN_REG_I];                         
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
                while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 2, 2) == HAL_BUSY);
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
              
                // 2. Выбор внешней опоры REFP0/REFN0
                hadc.REGISTERS[ADS1220_VREF_REG_I] &= ~ADS1220_VREF_MASK;
                hadc.REGISTERS[ADS1220_VREF_REG_I] |= ADS1220_VREF_REF0; // reference VREF 0                
                hadc.TXbuf[0] = ADS1220_CMD_WREG | ADS1220_VREF_REG;
                hadc.TXbuf[1] = hadc.REGISTERS[ADS1220_VREF_REG_I];              
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
                while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 2, 2) == HAL_BUSY);          
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);                                
              
                // 3. Выбор измерительных входов AIN0 (+) и AIN1 (-)
                hadc.REGISTERS[ADS1220_MUX_REG_I] &= ~ADS1220_MUX_MASK;              
                hadc.REGISTERS[ADS1220_MUX_REG_I] |= ADS1220_MUX_01; // AIN0+AIN1                
                hadc.TXbuf[0] = ADS1220_CMD_WREG | ADS1220_MUX_REG;
                hadc.TXbuf[1] = hadc.REGISTERS[ADS1220_MUX_REG_I];              
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);                
                while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 2, 2) == HAL_BUSY);  
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
                              
                // 4. Запуск преобразований (Команда START)
                hadc.TXbuf[0] = ADS1220_CMD_START;
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
                while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 1, 2) == HAL_BUSY);
                HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
              
                hadc.ADC_STATE = stADC_CH0;
                break;
            }
          case stADCConfigureForCalibration:
          {
              /*
              hadc.REGISTERS[ADS1220_TS_REG_I] &= ~ADS1220_TS_MASK;
              hadc.REGISTERS[ADS1220_TS_REG_I] |= ADS1220_TS_OFF;    //TS OFF                
              hadc.TXbuf[0] = ADS1220_CMD_WREG | ADS1220_TS_REG;
              hadc.TXbuf[1] = hadc.REGISTERS[ADS1220_TS_REG_I];
              HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
              while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 2, 2) == HAL_BUSY); 
              HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
              */
              
              hadc.REGISTERS[ADS1220_MUX_REG_I] &= ~ADS1220_MUX_MASK;
              hadc.REGISTERS[ADS1220_MUX_REG_I] |= ADS1220_MUX_ZCAL;
              hadc.TXbuf[0] = ADS1220_CMD_WREG | ADS1220_MUX_REG;
              hadc.TXbuf[1] = hadc.REGISTERS[ADS1220_MUX_REG_I];    //zero calibration
              HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
              while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 2, 2) == HAL_BUSY);         
              HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
                  
              hadc.CalibSum = 0;
              hadc.CalibCounter = 0;                    
            
              hadc.TXbuf[0] = ADS1220_CMD_START;
              HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
              while(HAL_SPI_Transmit(hadc.InterfaceHandle, hadc.TXbuf, 1, 2) == HAL_BUSY);
              HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
              
              hadc.ADC_STATE = stADCCalibration;
              break;
          }
          
          case stADCCalibration:
          {
              PrevRDYPin = RDYPin;
              RDYPin = HAL_GPIO_ReadPin(hadc.RDYPort, hadc.RDYPin);
              if((!RDYPin)&&(PrevRDYPin))
              {                   
                  hadc.TXbuf[0] = ADS1220_DUMMY;
                  hadc.TXbuf[1] = ADS1220_DUMMY;
                  hadc.TXbuf[2] = ADS1220_DUMMY;
                  hadc.NewDataAvailable = 0;
                  HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
                  while(HAL_SPI_TransmitReceive(hadc.InterfaceHandle, hadc.TXbuf, hadc.RXbuf, 3, 2) == HAL_BUSY);   
                  HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
              //}
              
              //if(hadc[adc].NewDataAvailable)
              //{            
                  //hadc[adc].NewDataAvailable = 0;
                  hadc.LastUpdate = GetUpTime();               
                  uint32_t tempval = (hadc.RXbuf[0] << 16) + (hadc.RXbuf[1] << 8) + hadc.RXbuf[2];
                  if(tempval & 0x00800000)
                  {
                      tempval |= 0xFF000000;
                  }
                  hadc.CalibSum += *(int32_t*)&tempval;
                  hadc.CalibCounter++;
                  if(hadc.CalibCounter >= 8)
                  {
                      hadc.Offset = hadc.CalibSum / 8; // average
                      hadc.ADC_STATE = stADCConfigureForCH0;
                  }
              }
              break;
          }
          case stADC_CH0: //measuring main
          {   
              PrevRDYPin = RDYPin;
              RDYPin = HAL_GPIO_ReadPin(hadc.RDYPort, hadc.RDYPin);                
              if((!RDYPin)&&(PrevRDYPin))
              {   
                  hadc.TXbuf[0] = ADS1220_DUMMY;
                  hadc.TXbuf[1] = ADS1220_DUMMY;
                  hadc.TXbuf[2] = ADS1220_DUMMY;
                  hadc.NewDataAvailable = 0;
                  HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_RESET);
                  //while(HAL_SPI_TransmitReceive_DMA(hadc[adc].InterfaceHandle, hadc[adc].TXbuf, hadc[adc].DMAbuf, 3) == HAL_BUSY);   
                  while(HAL_SPI_TransmitReceive(hadc.InterfaceHandle, hadc.TXbuf, hadc.RXbuf, 3, 2) == HAL_BUSY);   
                  HAL_GPIO_WritePin(hadc.CSPort, hadc.CSPin, GPIO_PIN_SET);
              //}
            
              //if(hadc[adc].NewDataAvailable)
              //{           
                  //hadc[adc].NewDataAvailable = 0;
                  hadc.LastUpdate = GetUpTime();               
                  uint32_t tempval = (hadc.RXbuf[0] << 16) + (hadc.RXbuf[1] << 8) + hadc.RXbuf[2];
                  if(tempval & 0x00800000)
                  {
                      tempval |= 0xFF000000;
                  }
                  
                  int32_t tempsigned =  *(int32_t*)&tempval - hadc.Offset;
                            
                  hadc.Ch0 = tempsigned;                    

                  float R_pt1000 = (-(int64_t)hadc.Ch0 * 10000) / 33554432.0f;

                // --- ТОЧНЫЙ РАСЧЕТ ТЕМПЕРАТУРЫ (МТШ-90 / Каллендар-Ван Дузен) ---
                // Константы для стандартной платины Pt1000 выше 0°C
                const float A = 3.9083e-3f;
                const float B = -5.775e-7f;

                // Раскрытое и упрощенное подкоренное выражение, чтобы МК не делал лишней работы:
                // под корнем получается: A^2 - 4*B + (4*B / 1000) * R
                float sqrt_val = 1.7584808e-5f - (2.310e-9f * R_pt1000);

                // Итоговая температура в градусах Цельсия
                float Temperature = (-A + sqrtf(sqrt_val)) / (2.0f * B);

                // Записываем результат в вашу переменную
                Pt1000Temp = Temperature;
                  
                  if(hadc.LastUpdate >= hadc.NextCalibration)
                  {
                      hadc.NextCalibration += 20000;
                      hadc.ADC_STATE = stADCConfigureForCalibration;
                  }                    
              }                           
              break;
          }
      }
      
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint64_t GetUpTime(void)
{
    volatile uint64_t res = uptime;
    while (res != uptime)
    {
        res = uptime;
    }
    return res;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &PC_UART)
    {
        PC_RX_flag = Size;
    }
    if(huart == &INFICON_UART)
    {
        HAL_UARTEx_ReceiveToIdle_IT(&INFICON_UART, INFICONRXBUF, sizeof INFICONRXBUF);
        InficonRX = Size;
    }
    if(huart == &LASER_UART)
    {
        
    }
}

void MemoryWrite(uint16_t memAddr, int32_t fdata) 
{
    uint16_t devAddr = (0x50 << 1);
    HAL_StatusTypeDef status;
    HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&fdata, 4, 4);
    for(;;) 
    { 
        status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1,  HAL_MAX_DELAY);
        if(status == HAL_OK)
        {
            break;
        }
    }
}

int32_t MemoryRead(uint16_t memAddr) 
{
    uint16_t devAddr = (0x50 << 1);
    uint8_t rmsg[4] = {0};
    HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)rmsg, 4, 4);
    int32_t fdata = *(int32_t*)&rmsg;
	return fdata;
}

void ParseInficon(uint8_t* buf, inficon_t* res, uint8_t full)
{
    res->Status = buf[2];
    if(buf[2] & 0x80)
    {
        res->Ready = 1;
    }else{
        res->Ready = 0;
    }
    res->FactorB = 32767;
    if(full)
    {   
        if((buf[1] == 2) || (buf[1] == 3))
        {
            switch((buf[2] & 0x30) >> 4)
            {
                case 0:
                {
                    res->Unit = unit_mbar;
                    res->FactorA = 1.3332f;
                    res->FactorB = 24000;
                    break;
                }
                case 1:
                {
                    res->Unit = unit_Torr;
                    res->FactorA = 1;
                    res->FactorB = 32000;
                    break;
                }
                case 2:
                {
                    res->Unit = unit_Pa;
                    res->FactorA = 133.32f;
                    res->FactorB = 24000;
                    break;
                }
                default:
                {
                    res->Unit = unit_mbar;
                    res->FactorA = 1.3332f;
                    res->FactorB = 24000;
                    break;
                }
            }
        }else{
            res->FactorB = 32767;
        }

        switch(buf[7] & 0x0F)
        {
            case 0x0:
            {
                res->FSR_Exp = 0.001f;
                break;
            }
            case 0x1:
            {
                res->FSR_Exp = 0.01f;
                break;
            }
            case 0x2:
            {
                res->FSR_Exp = 0.1f;
                break;
            }
            case 0x3:
            {
                res->FSR_Exp = 1;
                break;
            }
            case 0x4:
            {
                res->FSR_Exp = 10.0f;
                break;
            }
            case 0x5:
            {
                res->FSR_Exp = 100.0f;
                break;
            }
            case 0x6:
            {
                res->FSR_Exp = 1000.0f;
                break;
            }
            case 0x7:
            {
                res->FSR_Exp = 10000.0f;
                break;
            }
        }        
        switch(buf[7] & 0xF0)
        {
            case 0x00:
            {
                res->FSR_Mantissa = 1.0f;
                break;
            }
            case 0x10:
            {
                res->FSR_Mantissa = 1.1f;
                break;
            }
            case 0x20:
            {
                res->FSR_Mantissa = 2.0f;
                break;
            }
            case 0x30:
            {
                res->FSR_Mantissa = 2.5f;
                break;
            }
            case 0x40:
            {
                res->FSR_Mantissa = 5.0f;
                break;
            }
        }        
    }        
    res->Raw = buf[4] * 256 + buf[5];
    res->Pressure = (res->Raw * res->FactorA / res->FactorB) * res->FSR_Mantissa * res->FSR_Exp;
    if(res->Unit == unit_Torr)
    {
        res->PascalPressure = res->Pressure * 133.32f;
    }else{
        if(res->Unit == unit_mbar)
        {
            res->PascalPressure = res->Pressure * 1.3332f;
        }else{
            res->PascalPressure = res->Pressure;
        }
    }
    res->PascalSum -= res->Pascals[res->PascalIdx];
    res->PascalSum += res->PascalPressure;
    res->Pascals[res->PascalIdx] = res->PascalPressure;
    res->PascalIdx++;
    if(res->PascalIdx >= PASCAL_SIZE)
    {
        res->PascalIdx = 0;
    }            
    res->PascalPressureFiltered = res->PascalSum / PASCAL_SIZE;
    return;
}

void CommandExecute()
{
    static uint8_t tempbuf[COMMAND_LENGTH] = {0};
    tempbuf[0] = SOF;
    tempbuf[1] = command_buf[1];
    switch(command_buf[1])
    {
        case 0x00:
        {
            float fw = FWREV;
            uint32_t Seconds = GetUpTime() / 1000;
            memcpy(tempbuf + 2, &fw, 4);
            memcpy(tempbuf + 7, &Seconds, 4);
            memcpy(tempbuf + 11, &StatusRegister, 2);
            StatusRegister &= ~0x07;
            break;
        }
        case 0x01:
        {            
            memcpy(tempbuf + 2, &I.PascalPressureFiltered, 4);
            tempbuf[6] = I.OK;
            break;
        }        
        case 0x04:
        {
            memcpy(tempbuf + 2, &I420, 4);
            tempbuf[6] = Status420;
            break;
        }    
        case 0x06:
        {
            memcpy(tempbuf + 2, &Pt1000Temp, 4);            
            break;
        }          
        case 0x08:
        {
            PowerOffAllowed = 1;
            break;
        }
        case 0x09:
        {
            memcpy(&Venting, command_buf + 2, 2);
            memcpy(tempbuf + 2, command_buf + 2, 2);
            break;
        }
        case 0x0A:
        {            
            memcpy(tempbuf + 2, &Venting, 2);
            break;
        }
        case 0x0B:
        { 
            uint8_t _mode = command_buf[4];
            switch(_mode)
            {
                case 0: //stop
                {
                    if(RunButtonState == 0)
                    {
                        if(command_buf[3] != 0)
                        {
                            AccTime = command_buf[3];
                        }
                        if((command_buf[2] >= RPM_MIN) && (command_buf[2] <= RPM_MAX))
                        {
                            NewSpeedSetPoint = 0;
                        }
                    }
                    break;
                }
                case 1: //start
                {
                    if(RunButtonState == 0)
                    {
                        if(command_buf[3] != 0)
                        {
                            AccTime = command_buf[3];                        
                        }
                        if((command_buf[2] >= RPM_MIN) && (command_buf[2] <= RPM_MAX))
                        {
                            NewSpeedSetPoint = command_buf[2];
                        }
                    }
                    break;
                }
                case 2: //config
                {
                    if((command_buf[2] >= RPM_MIN) && (command_buf[2] <= RPM_MAX))
                    {
                        DefaultSpeedSetPoint = command_buf[2];
                    }
                    if(command_buf[3] != 0)
                    {
                        DefaultAccTime = command_buf[3];                        
                    }
                    break;
                }
            }
            memcpy(tempbuf + 2, command_buf + 2, 3);
            break;
        }
        case 0x0C:
        {            
            tempbuf[2] = (uint8_t)RawCurrentSpeed;
            break;
        }        
    }
    HAL_UART_Transmit_IT(&PC_UART, tempbuf, sizeof tempbuf);
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c == &hi2c2)
    {
        DataReady420 = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

