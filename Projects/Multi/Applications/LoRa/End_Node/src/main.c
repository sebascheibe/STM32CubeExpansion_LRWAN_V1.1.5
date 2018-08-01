/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.5
  * @date    30-March-2018
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "main.h"
#include "stm32l0xx_ll_utils.h"

//#include "tm_stm3210_gps.h"
//#include "tm_stm3210_gps.c"
//#include "tm_stm32_buffer.h"
//#include "tm_stm32_usart.h"
//#include "stm32l0xx_hal_usart.h"
//#include "stm32l0xx_hal_usart.h"
//#include "stm32l0xx.h"
//#include "stm32l1xx_usart.h"
//#include "misc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*!
 * CAYENNE_LPP is myDevices Application server.
 */
#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_DATATYPE_GPS      			0x88
#define LPP_APP_PORT 91
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            4000
#define PHY_DUTY_CYCLE 																0


/*
#define UART												USART2
#define UART_TX_PIN									GPIO_PIN_2
#define UART_RX_PIN									GPIO_PIN_3
#define UART_GPIO_PORT							GPIOA 
#define UART_AF											GPIO_AF4_USART2
#define UART_CLK_ENABLE()						__USART2_CLK_ENABLE();
*/
//Uart Handle
//static UART_HandleTypeDef uartHandle;
//void GNSS_Uart_IoInit(void);

#define RX_BUFFER_SIZE   80 //12

/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON //ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           42 //64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );
	
/* LoRa endNode send request*/
static void Send( void );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent( void );

/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;

//globally available GPS variables:
float longitude = 0, latitude = 0, altitude = 0;
float direction = 0, velocity = 0;
	
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;
                                               
static TimerEvent_t TxTimer;

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent( void );
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK};

 
#define FIELD_MAX 80
 
void ProcessNMEALine(char *s)
{
  char *field[FIELD_MAX];
  int f;
  int i;
  char c;
  int x;
 
  //if (Verbose)
  //{
    puts("DecodeNMEA");
    printf("%s\n",s);
  //}
 
  // Check the line synchronization
 
  if (s[0] != '$')
    return;
 
  // Verify the line checksum integrity
 
  c = 0; // checksum
 
  i = 1; // Xor bytes between $ and *, but not including those bytes
 
  while((s[i] != 0) && (s[i] != '*'))
    c ^= s[i++];
 
  if (s[i] == 0)
    return;
 
  sscanf(&s[i + 1],"%x",&x); // Checksum byte - Note sscanf needs this to be an int, rather than a single byte
 
  if (c != (char)(x & 0xFF)) // Leave if checksum fails
    return;
 
  // Parse out fields on , and *
 
  f = 0;
 
  while(1)
  {
    field[f++] = s;
 
    while((*s != 0) && (*s != ',') && (*s != '*') && (*s != 0x0D) && (*s != 0x0A))
      s++;
 
    if ((*s == 0) || (*s == '*') || (*s == 0x0D) || (*s == 0x0A) || (f == (FIELD_MAX - 1)))
    {
      *s = 0;
      field[f] = NULL;
      break;
    }
 
    *s++ = 0;
  }
 
  //if (Verbose)
  //{
  //  printf("Fields %02d\n",f);
 
  //  for(i=0; i<f; i++)
  //    printf("#%02d : %s\n",i,field[i]);
  //}
 
  // Process a couple of NMEA sentences for illustration
 
  if ((strcmp(field[0],"$GPGLL") == 0) && (f > 6)) // Geographic Position, Latitude, Longitude and Time
  {
    double lat, lon;
    int lat_deg, lon_deg;
    double lat_min, lon_min;
    double fix_time;
    int fix_hour, fix_minute;
    double fix_second;
    char lat_hemi, lon_hemi, valid;
 
    // Field 1 Latitude DDMM.mmmmmm
    // Field 2 Lat Hemi N/S
    // Field 3 Longitude DDMMM.mmmmm
    // Field 4 Lon Hemi E/W
    // Field 5 UTC Time HHMMSS.SSS
    // Field 6 Fix A=Valid, V=Not Valid
    // Field 7 Mode A=Autonomous, D=Differential, E=Estimated, N=Not Valid [Optional] - Simulator Does not report this
 
    sscanf(field[1],"%lf",&lat);
 
    lat_hemi = field[2][0];
 
    sscanf(field[3],"%lf",&lon);
 
    lon_hemi = field[4][0];
 
    sscanf(field[5],"%lf",&fix_time);
 
    valid = field[6][0];
 
    if (valid == 'A')
    {
      // Extract Time-of-Fix
 
      fix_minute = (int)fix_time / 100;
 
      fix_second = fix_time - (fix_minute * 100);
 
      fix_hour = fix_minute / 100;
 
      fix_minute = fix_minute % 100;
 
      // Process Latitude DDMM.mmmmm
 
      lat_deg = (int)lat / 100; // Decompose NMEA form ASCII into DEGREES and MINUTES
 
      lat_min = lat - (lat_deg * 100);
 
      lat = (double)lat_deg + (lat_min / 60.0); // Computed Latitude in DECIMAL DEGREES
      latitude = lat;
	
      if (lat_hemi == 'S')
      {
        lat_deg = -lat_deg;
        lat = -lat;
      }
 
      // Process Longitude DDDMM.mmmmm
 
      lon_deg = (int)lon / 100; // Decompose NMEA form ASCII into DEGREES and MINUTES
 
      lon_min = lon - (lon_deg * 100);
 
      lon = (double)lon_deg + (lon_min / 60.0); // Computed Longitude in DECIMAL DEGREES
 
			longitude = lon;
			
      if (lon_hemi == 'W')
      {
        lon_deg = -lon_deg;
        lon = -lon;
      }
 
      printf("%4d %9.6lf %4d %9.6lf [%+14.10lf %+14.10lf] @ %02d:%02d:%06.3lf\n",
        lat_deg, lat_min, lon_deg, lon_min, lat, lon, fix_hour, fix_minute, fix_second );
    }
    else
      puts("Invalid Fix");
  }
  else if ((strcmp(field[0],"$GPGSA") == 0) && (f > 17)) // GPS DOP and Active Satellites
  {
    char mode, fix;
    double pdop, hdop, vdop;
    int i, sv;
 
    // Field 1 A=Automatic(3D/2D), M=Manual
    // Field 2 Fix 1=No Fix, 2=2D, 3=3D
    // Field 3 SV List#1
    // Field 14 SV List#12
    // Field 15 PDOP
    // Field 16 HDOP
    // Field 17 VDOP
 
    mode = field[1][0];
    fix = field[2][0];
 
    sscanf(field[15],"%lf",&pdop); // Position Dilution of precision (PDOP)
    sscanf(field[16],"%lf",&hdop); // Horizontal Dilution of precision (HDOP)
    sscanf(field[17],"%lf",&vdop); // Vertical Dilution of precision (VDOP)
 
    switch(mode)
    {
      case 'A' : puts("Mode : Automatic"); break;
      case 'M' : puts("Mode : Manual"); break;
      default : puts("Mode : Unknown");
    }
 
    switch(fix)
    {
      case '1' : puts("Fix : Not Available"); break;
      case '2' : puts("Fix : 2D"); break;
      case '3' : puts("Fix : 3D"); break;
      default : puts("Fix : Unknown");
    }
 
    printf("SV :");
 
    for(i=0; i<12; i++)
    {
      if (field[3+i][0])
      {
        sscanf(field[3+i],"%d",&sv);
        printf(" %3d", sv);
      }
    }
 
    putchar('\n');
    printf("PDOP : %5.2lf, HDOP : %5.2lf, VDOP : %5.2lf\n", pdop, hdop, vdop);
  }
}


/**
  * @brief RX buffers for storing received data
  */
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBufferB[RX_BUFFER_SIZE];
__IO uint32_t     uwNbReceivedChars;
__IO uint32_t     uwBufferReadyIndication;
uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;

/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config_2(void);
void     Configure_USART(void);
void     StartReception(void);
void     HandleContinuousReception(void);
//void     UserLED_Init(void);
//void     UserLED_Off(void);
//void     UserLED_Blinking(uint32_t Period);
void     UserButton_Init(void);
void     WaitForUserButtonPress(void);
void     PrintInfo(uint8_t *String, uint32_t Size);
void     UserDataTreatment(uint8_t *DataBuffer, uint32_t Size);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
	
	/* Variables used */
  //TM_GPS_Data_t GPS_Data;
  //TM_GPS_Result_t result, current;
  //TM_GPS_Float_t GPS_Float;
  //TM_GPS_Distance_t GPS_Distance;
  //char buffer[40];
	
  /* STM32 HAL library initialization*/
  HAL_Init();
  
  /* Configure the system clock*/
  SystemClock_Config();
  //SystemClock_Config_2();
  /* Configure the debug mode*/
  DBG_Init();
  
  /* Configure the hardware*/
  HW_Init();
	
  /* USER CODE BEGIN 1 */
	
	//Initialize LED2
  //UserLED_Init();
	//Set LED2 Off
  //UserLED_Off();
	//Initialize User push-button in EXTI mode
  UserButton_Init();
	//Configure USARTx (USART IP configuration and related GPIO initialization) */
  Configure_USART();
	//Wait for User push-button press to start transfer */
  //WaitForUserButtonPress();
	
	
	/* USER CODE END 1 */
  
  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  PRINTF("VERSION: %X\n\r", VERSION);
  
  LORA_Join();
  
  LoraStartTx( TX_ON_TIMER) ;
	
  PRINTF("GPS sensor start\n\r");
	//Initiate Continuous reception */
  StartReception();
	
  while( 1 )
  {
		//readData();
//		if (line_valid)
//    {
//      ProcessNMEALine(line_buffer);
//      line_valid = 0;
//    }
    //DISABLE_IRQ( );
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending 
     * and cortex will not enter low power anyway  */

#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower( );
#endif

    //ENABLE_IRQ();
    
    /* USER CODE BEGIN 2 */

		//Handle Continuous reception
    HandleContinuousReception();
		
    /* USER CODE END 2 */
 }
}

/**
  * @brief  This function configures USARTx Instance.
  * @note   This function is used to :
  *         -1- Enable GPIO clock and configures the USART pins.
  *         -2- NVIC Configuration for USART interrupts.
  *         -3- Enable the USART peripheral clock and clock source.
  *         -4- Configure USART functional parameters.
  *         -5- Enable USART.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_USART(void)
{

  /* (1) Enable GPIO clock and configures the USART pins *********************/

  /* Enable the peripheral clock of GPIO Port */
  USARTx_GPIO_CLK_ENABLE();

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  USARTx_SET_TX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_MODE_ALTERNATE);
  USARTx_SET_RX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_PULL_UP);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(USARTx_IRQn, 0);  
  NVIC_EnableIRQ(USARTx_IRQn);

  /* (3) Enable USART peripheral clock and clock source ***********************/
  USARTx_CLK_ENABLE();

  /* Set clock source */
  USARTx_CLK_SOURCE();

  /* (4) Configure USART functional parameters ********************************/
  
  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USARTx_INSTANCE);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USARTx_INSTANCE, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USARTx_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USARTx_INSTANCE, LL_USART_HWCONTROL_NONE);

  /* Oversampling by 16 */
  /* Reset value is LL_USART_OVERSAMPLING_16 */
  // LL_USART_SetOverSampling(USARTx_INSTANCE, LL_USART_OVERSAMPLING_16);

  /* Set Baudrate to 115200 using APB frequency set to 16000000 Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
  
      In this example, Peripheral Clock is expected to be equal to 16000000 Hz => equal to SystemCoreClock
  */
  LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock, LL_USART_OVERSAMPLING_16, 9600); //115200 

  /* (5) Enable USART *********************************************************/
  LL_USART_Enable(USARTx_INSTANCE);
  
  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USARTx_INSTANCE))) || (!(LL_USART_IsActiveFlag_REACK(USARTx_INSTANCE))))
  { 
  }
	PRINTF("Configure_USART Done \n\r");
}

/**
  * @brief  This function prints user info on PC com port and initiates RX transfer
  * @param  None
  * @retval None
  */
void StartReception(void)
{
  /* Initializes Buffer swap mechanism :
     - 2 physical buffers aRXBufferA and aRXBufferB (RX_BUFFER_SIZE length)
     
  */
  pBufferReadyForReception = aRXBufferA;
  pBufferReadyForUser      = aRXBufferB;
  uwNbReceivedChars = 0;
  uwBufferReadyIndication = 0;

  /* Print user info on PC com port */
  //PrintInfo(aTextInfoStart, sizeof(aTextInfoStart));

  /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(USARTx_INSTANCE);

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
  LL_USART_EnableIT_ERROR(USARTx_INSTANCE);
	
	PRINTF("Start_Reception Done \n\r");
}

/**
  * @brief  This function monitors buffer filling indication and calls User callbacks when a buffer is full
  * @param  None
  * @retval None
  */
void HandleContinuousReception(void)
{
  /* Checks if Buffer full indication has been set */
  if (uwBufferReadyIndication != 0)
  {
    /* Reset indication */
    uwBufferReadyIndication = 0;

    /* Call user Callback in charge of consuming data from filled buffer */
		PRINTF("Handle_Continuous_Reception Buffer \n\r"); 
    UserDataTreatment(pBufferReadyForUser, RX_BUFFER_SIZE);
		
  }
	//PRINTF("Handle_Continuous_Reception Done \n\r"); 
}

/**
  * @brief  Initialize LED2.
  * @param  None
  * @retval None
  */
/* void UserLED_Init(void)
{
  // Enable the LED2 Clock
  LED2_GPIO_CLK_ENABLE();

  // Configure IO in output push-pull mode to drive external LED2
  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
  // Reset value is LL_GPIO_OUTPUT_PUSHPULL
  //LL_GPIO_SetPinOutputType(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  // Reset value is LL_GPIO_SPEED_FREQ_LOW 
  //LL_GPIO_SetPinSpeed(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_LOW);
  // Reset value is LL_GPIO_PULL_NO
  //LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
}*/

/**
  * @brief  Turn-off LED2.
  * @param  None
  * @retval None
  */
/*
void UserLED_Off(void)
{
  // Turn LED2 off  
	LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}*/

/**
  * @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
  * @param  Period : Period of time (in ms) between each toggling of LED
  *   This parameter can be user defined values. Pre-defined values used in that example are :
  *     @arg LED_BLINK_FAST : Fast Blinking
  *     @arg LED_BLINK_SLOW : Slow Blinking
  *     @arg LED_BLINK_ERROR : Error specific Blinking
  * @retval None
  */
/*
void UserLED_Blinking(uint32_t Period)
{
  // Toggle LED2 in an infinite loop
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);  
    LL_mDelay(Period);
  }
}
*/
/**
  * @brief  Configures User push-button in GPIO or EXTI Line Mode.
  * @param  None 
  * @retval None
  */
void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();
  
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);

  /* Connect External Line to the GPIO*/
  USER_BUTTON_SYSCFG_SET_EXTI();

  /* Enable a rising trigger External lines 4 to 15 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 3);  
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn); 
	PRINTF("User_Button_Init Done \n\r");
}

/**
  * @brief  Wait for User push-button press to start transfer.
  * @param  None 
  * @retval None
  */
  /*  */
void WaitForUserButtonPress(void)
{
  while (ubButtonPress == 0)
  {
    //LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    //LL_mDelay(LED_BLINK_FAST);
		//RINTF("User_Button WAIT \n\r");
  }
  /* Ensure that LED2 is turned Off */
  //UserLED_Off();
}

/**
  * @brief  Send Txt information message on USART Tx line (to PC Com port).
  * @param  None
  * @retval None
  */
void PrintInfo(uint8_t *String, uint32_t Size)
{
  uint32_t index = 0;
  uint8_t *pchar = String;
  
  /* Send characters one per one, until last char to be sent */
  for (index = 0; index < Size; index++)
  {
    /* Wait for TXE flag to be raised */
    while (!LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE))
    {
    }

    /* Write character in Transmit Data register.
       TXE flag is cleared by writing data in TDR register */
    LL_USART_TransmitData8(USARTx_INSTANCE, *pchar++);
  }

  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USARTx_INSTANCE))
  {
  }
}

/**
  * @brief  Example of User callback in charge of consuming received data.
  * @param  None
  * @retval None
  */
void UserDataTreatment(uint8_t *DataBuffer, uint32_t Size)
{
	PRINTF("User_Data_Treatment Begin \n\r");
  /* Toggle LED */
  //LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
  //char nmealine[Size+1];
	//char *p;
	//for(int i=0; i < Size; i++)
	//{
	//	nmealine[i] = (char)DataBuffer;
	//}
	//p = nmealine;
  //TODO parse NMEA string
	//ProcessNMEALine(p);
	ProcessNMEALine((char*)DataBuffer);
	printf("%s",DataBuffer);
	PRINTF("User_Data_Treatment Done \n\r");
}

/**
  * @brief  System Clock Configuration
      *         The system Clock is configured as follows :
      *            System Clock source            = HSI RC
      *            SYSCLK(Hz)                     = 16000000
      *            HCLK(Hz)                       = 16000000
      *            AHB Prescaler                  = 1
      *            APB1 Prescaler                 = 1
      *            APB2 Prescaler                 = 1
      *            HSI Frequency(Hz)              = 16000000
      *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */
void SystemClock_Config_2(void)
{
    LL_RCC_PLL_Disable();
    // Set new latency
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
 
    // HSI configuration and activation
    LL_RCC_HSI_Enable();
    LL_RCC_HSI_DisableDivider();
    while(LL_RCC_HSI_IsReady() != 1) 
    {
    };
    
  // Sysclk activation on the HSI
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) 
  {
  };
  
  // Set AHB & APB1 & APB2 prescaler
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  // Disable MSI
  LL_RCC_MSI_Disable();
  while(LL_RCC_MSI_IsReady() != 0) 
  {
  };

  // Set systick to 1ms in using frequency set to 16MHz
  //LL_Init1msTick(16000000);
  
  // Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function)
  //LL_SetSystemCoreClock(16000000);
	PRINTF("System_Clock_Init_2 Done \n\r");
	
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT Functions                                     */
/******************************************************************************/
/**
  * @brief  Function to manage User push-button
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Update User push-button variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;
}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
uint8_t *ptemp;

  // Read Received character. RXNE flag is cleared by reading of RDR register
  pBufferReadyForReception[uwNbReceivedChars++] = LL_USART_ReceiveData8(USARTx_INSTANCE);

  // Checks if Buffer full indication has been set
  if (uwNbReceivedChars >= RX_BUFFER_SIZE || LL_USART_ReceiveData8(USARTx_INSTANCE) == '\r')
  {
    // Set Buffer swap indication
    uwBufferReadyIndication = 1;

    // Swap buffers for next bytes to be received
    ptemp = pBufferReadyForUser;
    pBufferReadyForUser = pBufferReadyForReception;
    pBufferReadyForReception = ptemp;
    uwNbReceivedChars = 0;
		PRINTF("USART_CharReception_Callback Switch Buffers! New Line symbol\n\r");
  }
	else if(LL_USART_ReceiveData8(USARTx_INSTANCE) == '$')
	{
		// Set Buffer swap indication
    uwBufferReadyIndication = 1;

    // Swap buffers for next bytes to be received
    ptemp = pBufferReadyForUser;
    pBufferReadyForUser = pBufferReadyForReception;
    pBufferReadyForReception = ptemp;
    uwNbReceivedChars = 1;
		pBufferReadyForReception[0] = '$'; //Set first character of new Buffer
		
		PRINTF("USART_CharReception_Callback Switch Buffers!NMEA start symbol \n\r");
	}
	PRINTF("USART_CharReception_Callback Done \n\r");
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
	PRINTF("Error_Callback Begin \n\r");
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USARTx_IRQn);
  
  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USARTx_INSTANCE, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : Clear NF Flag */
    LL_USART_ClearFlag_NE(USARTx_INSTANCE);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    //UserLED_Blinking(LED_BLINK_ERROR);
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


static void LORA_HasJoined( void )
{
#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
}

static void Send( void )
{
  /* USER CODE BEGIN 3 */
  //uint16_t pressure = 0;
  //int16_t temperature = 0;
  //uint16_t humidity = 0;
	//float latitude, longitude, altitude = 0;
	int32_t latitude_lpp, longitude_lpp, altitude_lpp = 0;
  //uint8_t batteryLevel;
  //sensor_t sensor_data;
  
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    return;
  }
  
  DBG_PRINTF("SEND REQUEST\n\r");
#ifndef CAYENNE_LPP
  int32_t latitude, longitude = 0;
  uint16_t altitudeGps = 0;
#endif
  
#ifdef USE_B_L072Z_LRWAN1
  TimerInit( &TxLedTimer, OnTimerLedEvent );
  
  TimerSetValue(  &TxLedTimer, 200);
  
  LED_On( LED_RED1 ) ; 
  
  TimerStart( &TxLedTimer );  
#endif

  //BSP_sensor_Read( &sensor_data );

#ifdef CAYENNE_LPP
  uint8_t cchannel=0;
  //temperature = ( int16_t )( sensor_data.temperature * 10 );     /* in °C * 10 */
  //pressure    = ( uint16_t )( sensor_data.pressure * 100 / 10 );  /* in hPa / 10 */
  //humidity    = ( uint16_t )( sensor_data.humidity * 2 );        /* in %*2     */
  
	//TODO retrieve sensor data
	//latitude = -31.7330;
	//longitude = -60.5297;
	//altitude = 7.40;
	PRINTF("GPS DATA \n\r");
	//PRINTF("LAT: %lf \n\r", latitude);
	//PRINTF("LON: %lf \n\r", longitude);
	//PRINTF("ALT: %lf \n\r", altitude);
	//PRINTF("velocity: %lf \n\r", velocity);
	//PRINTF("direction: %lf \n\r", direction);
	
	
	latitude_lpp = ( int32_t ) ( latitude * 10000 );
	longitude_lpp = ( int32_t ) ( longitude * 10000 );
	altitude_lpp = ( int32_t ) ( altitude * 100 );
	
  uint32_t i = 0;

  //batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LPP_APP_PORT;
/*
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_BAROMETER;
  AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  
	AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_TEMPERATURE; 
  AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_HUMIDITY;
  AppData.Buff[i++] = humidity & 0xFF;
*/
	AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_GPS;
	AppData.Buff[i++] = ( latitude_lpp >> 16 ) & 0xFF;
  AppData.Buff[i++] = ( latitude_lpp >> 8 ) & 0xFF;
  AppData.Buff[i++] = latitude_lpp & 0xFF;
  AppData.Buff[i++] = ( longitude_lpp >> 16 ) & 0xFF;
  AppData.Buff[i++] = ( longitude_lpp >> 8 ) & 0xFF;
  AppData.Buff[i++] = longitude_lpp & 0xFF;
	AppData.Buff[i++] = ( altitude_lpp >> 16 ) & 0xFF;
  AppData.Buff[i++] = ( altitude_lpp >> 8 ) & 0xFF;
  AppData.Buff[i++] = altitude_lpp & 0xFF;
	
	
#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 )
  /* The maximum payload size does not allow to send more data for lowest DRs */
#else
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_INPUT; 
  AppData.Buff[i++] = batteryLevel*100/254;
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_OUTPUT; 
  AppData.Buff[i++] = AppLedStateOn;
#endif  /* REGION_XX915 */
#else  /* not CAYENNE_LPP */

  temperature = ( int16_t )( sensor_data.temperature * 100 );     /* in °C * 100 */
  pressure    = ( uint16_t )( sensor_data.pressure * 100 / 10 );  /* in hPa / 10 */
  humidity    = ( uint16_t )( sensor_data.humidity * 10 );        /* in %*10     */
  latitude = sensor_data.latitude;
  longitude= sensor_data.longitude;
  uint32_t i = 0;

  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LORAWAN_APP_PORT;

#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 )
  AppData.Buff[i++] = AppLedStateOn;
  AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = ( humidity >> 8 ) & 0xFF;
  AppData.Buff[i++] = humidity & 0xFF;
  AppData.Buff[i++] = batteryLevel;
  AppData.Buff[i++] = 0;
  AppData.Buff[i++] = 0;
  AppData.Buff[i++] = 0;
#else  /* not REGION_XX915 */
  AppData.Buff[i++] = AppLedStateOn;
  AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = ( humidity >> 8 ) & 0xFF;
  AppData.Buff[i++] = humidity & 0xFF;
  AppData.Buff[i++] = batteryLevel;
  AppData.Buff[i++] = ( latitude >> 16 ) & 0xFF;
  AppData.Buff[i++] = ( latitude >> 8 ) & 0xFF;
  AppData.Buff[i++] = latitude & 0xFF;
  AppData.Buff[i++] = ( longitude >> 16 ) & 0xFF;
  AppData.Buff[i++] = ( longitude >> 8 ) & 0xFF;
  AppData.Buff[i++] = longitude & 0xFF;
  AppData.Buff[i++] = ( altitudeGps >> 8 ) & 0xFF;
  AppData.Buff[i++] = altitudeGps & 0xFF;
#endif  /* REGION_XX915 */
#endif  /* CAYENNE_LPP */
  AppData.BuffSize = i;
  
  LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
  
  /* USER CODE END 3 */
}


static void LORA_RxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
  DBG_PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case 3:
    /*this port switches the class*/
    if( AppData->BuffSize == 1 )
    {
      switch (  AppData->Buff[0] )
      {
        case 0:
        {
          LORA_RequestClass(CLASS_A);
          break;
        }
        case 1:
        {
          LORA_RequestClass(CLASS_B);
          break;
        }
        case 2:
        {
          LORA_RequestClass(CLASS_C);
          break;
        }
        default:
          break;
      }
    }
    break;
    case LORAWAN_APP_PORT:
    if( AppData->BuffSize == 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On( LED_BLUE ) ; 
      }
    }
    break;
  case LPP_APP_PORT:
  {
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
    if ( AppLedStateOn == RESET )
    {
      PRINTF("LED OFF\n\r");
      LED_Off( LED_BLUE ) ; 
      
    }
    else
    {
      PRINTF("LED ON\n\r");
      LED_On( LED_BLUE ) ; 
    }
    break;
  }
  default:
    break;
  }
  /* USER CODE END 4 */
}

static void OnTxTimerEvent( void )
{
  Send( );
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent();
  }
  else
  {
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct={0};
  
    initStruct.Mode =GPIO_MODE_IT_RISING;
    initStruct.Pull = GPIO_PULLUP;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
    HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded ( void )
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void )
{
  LED_Off( LED_RED1 ) ; 
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
