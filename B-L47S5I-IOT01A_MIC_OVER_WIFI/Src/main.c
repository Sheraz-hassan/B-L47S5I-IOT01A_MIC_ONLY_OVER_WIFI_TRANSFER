/**
  ******************************************************************************
  * @file    Wifi/WiFi_HTTP_Server/src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license SLA0044,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        http://www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#ifdef __ICCARM__
#include <LowLevelIOInterface.h>
#endif

/* Private defines -----------------------------------------------------------*/

/*IP AND PORT INITIALIZATION*/ 

uint8_t RemoteIP[] = {192,168,137,1};   //IP ADDRESS OF THE SERVER YOU WANT TO CONNECT 
#define RemotePORT   8002               //PORT ADDRESS OF THE SERVER YOU WANT TO CONNECT 
//#define PASSWORD  "Sheraz12345"         //PASSWORD OF THE WIFI (IF NOT OPEN) YOU WANT TO CONNECT 
#define PASSWORD  "lums12345"

#define PORT           80
#define TERMINAL_USE
#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define SOCKET                 0

#ifdef  TERMINAL_USE
#define LOG(a) printf a
#else
#define LOG(a)
#endif

#define SSID_SIZE     100
#define PASSWORD_SIZE 100
#define USER_CONF_MAGIC                 0x0123456789ABCDEFuLL
#define CONNECTION_TRIAL_MAX          10
////////////////////////////

/*VARIABLES DEFINED FOR SENSORS USED*/ 

int32_t RecBuf[256];
int32_t PlayBuf[256];

uint8_t halfbufcheck=0;
uint8_t fullbufcheck=0;

uint8_t sound[1024];

/* Private typedef------------------------------------------------------------*/

typedef struct {
  char ssid[SSID_SIZE];
  char password[PASSWORD_SIZE];
  uint8_t security;
} wifi_config_t;

typedef struct {
  uint64_t      wifi_config_magic;        /**< The USER_CONF_MAGIC magic word signals that the wifi config
                                               (wifi_config_t) is present in Flash. */
  wifi_config_t wifi_config;
} user_config_t;

   int32_t Socket = -1;
  uint16_t Datalen;
  int32_t ret;
  int16_t Trials = CONNECTION_TRIAL_MAX;
   uint8_t Tim1[115];
  // uint8_t Tim2[91] = " ";
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */

/* configuration storage in Flash memory */
#if defined(__ICCARM__)
/* IAR */
extern void __ICFEDIT_region_FIXED_LOC_start__;
const  user_config_t    *lUserConfigPtr = &__ICFEDIT_region_FIXED_LOC_start__;
#elif defined(__CC_ARM)
/* Keil / armcc */
user_config_t __uninited_region_start__ __attribute__((section("UNINIT_FIXED_LOC"), zero_init));
const  user_config_t    *lUserConfigPtr = &__uninited_region_start__;
#elif defined(__GNUC__)
/* GNU compiler */
user_config_t __uninited_region_start__ __attribute__((section("UNINIT_FIXED_LOC")));
const  user_config_t    *lUserConfigPtr = &__uninited_region_start__;
#endif


/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);
static int wifi_start(void);
static int wifi_connect(void);

static void Button_ISR(void);
static void Button_Reset(void);
static uint8_t Button_WaitForPush(uint32_t delay);
static void MX_I2C2_Init(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DMA_Init(void);

static volatile uint8_t button_flag = 0;
static user_config_t user_config;

static  uint8_t  IP_Addr[4];
static  int     LedState = 0; 

I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim2;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


int main(void)
{
	int j=0;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure LED2 */
  BSP_LED_Init(LED2);
	
	/* Configure USER push button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);  //USER push button is used to ask if reconfiguration is needed
	
	/* Configure all I/O ports */
  MX_GPIO_Init();
  MX_I2C2_Init();

	MX_DMA_Init();
  MX_DFSDM1_Init();
   
  
	/* Start the MIC and get a reading */

	HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0,RecBuf,256); // Permanently start sound sensor 
   

  /* WIFI Server */
#if defined (TERMINAL_USE)
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;


  BSP_COM_Init(COM1, &hDiscoUart);
  

  printf("\n****** WIFI Server ******\n\n");

#endif /* TERMINAL_USE */

   wifi_connect(); //Establish a wifi connection
  while (Trials--)
  {
      if( WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
    {
         printf("> TCP Connection opened successfully.\n");
      Socket = 0;
      break;
      }
  }
	



  while(1)
  {        
     if(halfbufcheck==1)
     {		  
        for(int i=0;i<256/2;i++)
        {
           PlayBuf[i]=RecBuf[i]>>6;
					 sprintf(&sound[j], "(%.05li)", PlayBuf[i]);
					 j=j+8;

        }
				
				WIFI_SendData(Socket, sound, sizeof(sound), &Datalen, 0);
        halfbufcheck=0;
				j=0;				
     }
     if(fullbufcheck==1)
     {
			  for(int i=256/2;i<256;i++)
        {
           PlayBuf[i]=RecBuf[i]>>6;
					 sprintf(&sound[j], "(%.05li)", PlayBuf[i]);
					 j=j+8;
           
        }
				
				WIFI_SendData(Socket, sound, sizeof(sound), &Datalen, 0);		
				fullbufcheck=0;
				j=0;
     }
		 
  }

}

/**
  * @brief  Start Wifi
  * @param  None
  * @retval None
  */


static int wifi_start(void)
{
  uint8_t  MAC_Addr[6];

 /*Initialize and use WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    printf("eS-WiFi Initialized.\n");
    if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
    {
      LOG(("eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\n",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]));
    }
    else
    {
      LOG(("> ERROR : CANNOT get MAC address\n"));
      return -1;
    }
  }
  else
  {
    return -1;
  }
  return 0;
}



int wifi_connect(void)
{
  wifi_start();
  
  memset(&user_config, 0, sizeof(user_config));
  memcpy(&user_config, lUserConfigPtr, sizeof(user_config));
  if (user_config.wifi_config_magic == USER_CONF_MAGIC)
  {
    /* WiFi configuration is already in Flash. Ask if we want to change it */
    printf("Already configured SSID: %s security: %d\n",
           user_config.wifi_config.ssid, user_config.wifi_config.security);
    printf("Press board User button (blue) within 5 seconds if you want to change the configuration.\n");
    Button_Reset();
    if (Button_WaitForPush(5000))
    {
      /* we want to change the configuration already stored in Flash memory */
      memset(&user_config, 0, sizeof(user_config));
    }
  }

  if (user_config.wifi_config_magic != USER_CONF_MAGIC)
  {
    printf("\nEnter WiFi SSID : ");
    gets(user_config.wifi_config.ssid);
    LOG(("\nYou have entered %s as SSID.\n", user_config.wifi_config.ssid));

    char c;
    do
    {
        printf("\rEnter Security Mode (0 - Open, 1 - WEP, 2 - WPA, 3 - WPA2): ");
        c = getchar();
    }
    while ( (c < '0')  || (c > '3'));
    user_config.wifi_config.security = c - '0';
    LOG(("\nYou have entered %d as the security mode.\n", user_config.wifi_config.security));

    if (user_config.wifi_config.security != 0)
    {
      printf("\nUsing Password as entered in the code: ");
      //gets(user_config.wifi_config.password);
			strcpy(user_config.wifi_config.password,PASSWORD);
    }
    user_config.wifi_config_magic = USER_CONF_MAGIC;
    FLASH_Erase_Size((uint32_t)lUserConfigPtr, sizeof(user_config));
    FLASH_Write((uint32_t)lUserConfigPtr, (uint32_t*)&user_config, sizeof(user_config));
  }
  
  printf("\nConnecting to %s\n", user_config.wifi_config.ssid);
	
  WIFI_Ecn_t security;
	
  switch (user_config.wifi_config.security)
  {
    case 0:
      security = WIFI_ECN_OPEN;
      break;
    case 1:
      security = WIFI_ECN_WEP;
      break;
    case 2:
      security =  WIFI_ECN_WPA_PSK;
      break;
    case 3:
    default:
      security =  WIFI_ECN_WPA2_PSK;
      break;
  }
  if (WIFI_Connect(user_config.wifi_config.ssid, user_config.wifi_config.password, security) == WIFI_STATUS_OK)
  {
    if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
    {
      LOG(("eS-WiFi module connected: got IP Address : %d.%d.%d.%d\n",
               IP_Addr[0],
               IP_Addr[1],
               IP_Addr[2],
               IP_Addr[3]));
    }
    else
    {
      LOG((" ERROR : es-wifi module CANNOT get IP address\n"));
      return -1;
    }
  }
  else
  {
     LOG(("ERROR : es-wifi module NOT connected\n"));
     return -1;
  }
  return 0;
}


/*!!!!!!!!!!!!!!!!!DISCLAIMER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
//SYSTEM CLOCK CONFIGRATIONS MIGHT HAVE BEEN CHANGED TO GET BETTER RESULTS AND THE BRIEF BELOW MIGHT BE INACCURATE
// CHECK  SystemClock_Config(void) TO SEE THE ACTUAL VALUES
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLR_DIV2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 15;// Controls the distance coverage (Distance coverage decreases with it but sample rate increases)-My assumption
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 30;// Controls the clearness  (clearness decreases with it but sample rate increases)-My assumption
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 20;// Controls the sensitivity (sensitivity  decreases with it but sample rate increases)-My assumption
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

}
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
   halfbufcheck=1;
}
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
   fullbufcheck=1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM1) 
	{
		HAL_IncTick();
	}  

  /* USER CODE END Callback 0 */


  /* USER CODE END Callback 1 */
}


/**
  * @brief Reset button state
  *        To be called before Button_WaitForPush()
  */
void Button_Reset()
{
  button_flag = 0;
}


/**
  * @brief Waiting for button to be pushed
  */
uint8_t Button_WaitForPush(uint32_t delay)
{
  uint32_t time_out = HAL_GetTick() + delay;

  do
  {
    if (button_flag > 0)
    {
      return button_flag;
    }
    HAL_Delay(100);
  }
  while (HAL_GetTick() < time_out);

  return 0;
}

#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}


#ifdef __ICCARM__
/**
  * @brief  
  * @param  
  * @retval 
  */
size_t __read(int handle, unsigned char * buffer, size_t size)
{
  int nChars = 0;

  /* handle ? */

  for (/* Empty */; size > 0; --size)
  {
    uint8_t ch = 0;
    while (HAL_OK != HAL_UART_Receive(&hDiscoUart, (uint8_t *)&ch, 1, 30000))
    {
      ;
    }

    *buffer++ = ch;
    ++nChars;
  }

  return nChars;
}
#elif defined(__CC_ARM) || defined(__GNUC__)
/**
  * @brief  Retargets the C library scanf function to the USART.
  * @param  None
  * @retval None
  */
GETCHAR_PROTOTYPE
{
  /* Place your implementation of fgetc here */
  /* e.g. read a character on USART and loop until the end of read */
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&hDiscoUart, (uint8_t *)&ch, 1, 30000))
  {
    ;
  }
  return ch;
}
#endif /* defined(__CC_ARM)  */
#endif /* TERMINAL_USE */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (USER_BUTTON_PIN):
    {
      Button_ISR();
      break;
    }
    case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
    default:
    {
      break;
    }
  }
}

/**
  * @brief  SPI3 line detection callback.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

/**
  * @brief Update button ISR status
  */
static void Button_ISR(void)
{
  button_flag++;
}

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
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

