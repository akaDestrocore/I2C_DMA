/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t master_tx_buffer[32] = "Hello from I2C1 Master!";
uint8_t master_rx_buffer[32];
uint8_t slave_tx_buffer[32] = "Response from I2C2 Slave!";
uint8_t slave_rx_buffer[32];

volatile uint8_t master_tx_complete = 0;
volatile uint8_t master_rx_complete = 0;
volatile uint8_t slave_tx_complete = 0;
volatile uint8_t slave_rx_complete = 0;

// I2C slave addr
#define SLAVE_ADDRESS 0x50
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I2C1_Master_Transmit_DMA(uint8_t slaveAddr, uint8_t *pData, uint16_t size)
{
    // Set addr, size and buffer for DMA transfers
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)pData);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)&I2C1->TXDR);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, size);

    // Enable DMA req for I2C1
    LL_I2C_EnableDMAReq_TX(I2C1);

    // Enable IRQ
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);

    // Enable DMA channel
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);

    // Send start sequence and slave addr
    LL_I2C_HandleTransfer(I2C1, slaveAddr, LL_I2C_ADDRSLAVE_7BIT, size,
                          LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    master_tx_complete = 0;
}

void I2C1_Master_Receive_DMA(uint8_t slaveAddr, uint8_t *pData, uint16_t size)
{
    // Set addr, size and buffer for DMA reception
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_7, (uint32_t)pData);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)&I2C1->RXDR);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, size);

    // Enable DMA reqs for I2C1
    LL_I2C_EnableDMAReq_RX(I2C1);

    // Enable IRQ
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);

    // Enable DMA Channel
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);

    // Send start sequence and slave addr in reading mode
    LL_I2C_HandleTransfer(I2C1, slaveAddr, LL_I2C_ADDRSLAVE_7BIT, size,
                          LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    master_rx_complete = 0;
}

void I2C2_Slave_Init(void)
{
    // Enable interrupt for address matchig event
    LL_I2C_EnableIT_ADDR(I2C2);

    // clear slave addresses
    slave_tx_complete = 0;
    slave_rx_complete = 0;
}

void I2C2_Slave_Receive_DMA_Setup(uint8_t *pData, uint16_t size)
{
    // Set addr, size and buffer for DMA reception
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)pData);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&I2C2->RXDR);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, size);

    // Reset TC bit
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);

    // Enable DMA req for I2C2
    LL_I2C_EnableDMAReq_RX(I2C2);

    // Enable IRQ
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);

    // Enable DMA Channel
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);


    slave_rx_complete = 0;
}

void I2C2_Slave_Transmit_DMA_Setup(uint8_t *pData, uint16_t size)
{
    // Set addr, size and buffer for DMA transfers
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)pData);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&I2C2->TXDR);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, size);

    // Reset TC bit
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

    // Enable DMA req for I2C2
    LL_I2C_EnableDMAReq_TX(I2C2);

    // Enable IRQ
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

    // Enable DMA Channel
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);

    slave_tx_complete = 0;
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
 void DMA1_Channel4_IRQHandler(void)
 {
   /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */
   // I2C2 TX
   if (LL_DMA_IsActiveFlag_TC4(DMA1)) {
     LL_DMA_ClearFlag_TC4(DMA1);
     LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
     slave_tx_complete = 1;
   }
   /* USER CODE END DMA1_Channel4_IRQn 0 */
   /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

   /* USER CODE END DMA1_Channel4_IRQn 1 */
 }

 /**
   * @brief This function handles DMA1 channel5 global interrupt.
   */
 void DMA1_Channel5_IRQHandler(void)
 {
   /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
   // I2C2 RX
   if (LL_DMA_IsActiveFlag_TC5(DMA1)) {
     LL_DMA_ClearFlag_TC5(DMA1);
     LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
     slave_rx_complete = 1;
   }
   /* USER CODE END DMA1_Channel5_IRQn 0 */
   /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

   /* USER CODE END DMA1_Channel5_IRQn 1 */
 }

 /**
   * @brief This function handles DMA1 channel6 global interrupt.
   */
 void DMA1_Channel6_IRQHandler(void)
 {
   /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
   // I2C1 TX
   if (LL_DMA_IsActiveFlag_TC6(DMA1)) {
     LL_DMA_ClearFlag_TC6(DMA1);
     LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
     master_tx_complete = 1;
   }
   /* USER CODE END DMA1_Channel6_IRQn 0 */
   /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

   /* USER CODE END DMA1_Channel6_IRQn 1 */
 }

 /**
   * @brief This function handles DMA1 channel7 global interrupt.
   */
 void DMA1_Channel7_IRQHandler(void)
 {
   /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
   // I2C1 RX
   if (LL_DMA_IsActiveFlag_TC7(DMA1)) {
     LL_DMA_ClearFlag_TC7(DMA1);
     LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
     master_rx_complete = 1;
   }
   /* USER CODE END DMA1_Channel7_IRQn 0 */
   /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

   /* USER CODE END DMA1_Channel7_IRQn 1 */
 }

 /**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
 void I2C2_EV_IRQHandler(void)
 {
   if (LL_I2C_IsActiveFlag_ADDR(I2C2)) {

     LL_I2C_ClearFlag_ADDR(I2C2);

     if (LL_I2C_GetTransferDirection(I2C2) == LL_I2C_DIRECTION_WRITE) {

       LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
     } else {

       LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
     }
   }

   if (LL_I2C_IsActiveFlag_STOP(I2C2)) {

     LL_I2C_ClearFlag_STOP(I2C2);

     if (1 == slave_rx_complete) {

       I2C2_Slave_Receive_DMA_Setup(slave_rx_buffer, sizeof(slave_rx_buffer));
     }
     if (1 == slave_tx_complete) {

       I2C2_Slave_Transmit_DMA_Setup(slave_tx_buffer, sizeof(slave_tx_buffer));
     }
   }
 }

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */

  /* USER CODE END I2C2_ER_IRQn 0 */
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  I2C2_Slave_Init();

  // Prepare slave
  I2C2_Slave_Receive_DMA_Setup(slave_rx_buffer, sizeof(slave_rx_buffer));

  // Prepare slave to send data
  I2C2_Slave_Transmit_DMA_Setup(slave_tx_buffer, sizeof(slave_tx_buffer));

  // wait ?
  LL_mDelay(100);

  // Send from master to slave
  I2C1_Master_Transmit_DMA(SLAVE_ADDRESS, master_tx_buffer, strlen((char*)master_tx_buffer));

  while (1 != master_tx_complete){
    // wait TC
  }

  LL_mDelay(100);

  // Master accept data from slave
  I2C1_Master_Receive_DMA(SLAVE_ADDRESS, master_rx_buffer, strlen((char*)slave_tx_buffer));

  while (1 != master_rx_complete){
    // wait TC
  }

  LL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // send data periodically
    if (1 == master_tx_complete && 1 == master_rx_complete) {
      master_tx_complete = 0;
      master_rx_complete = 0;

      LL_mDelay(1000);

      I2C1_Master_Transmit_DMA(SLAVE_ADDRESS, master_tx_buffer, strlen((char*)master_tx_buffer));

      LL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);

      LL_mDelay(100);

      I2C1_Master_Receive_DMA(SLAVE_ADDRESS, master_rx_buffer, strlen((char*)slave_tx_buffer));
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB8   ------> I2C1_SCL
  PB9   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 DMA Init */

  /* I2C1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_7, LL_DMA_REQUEST_3);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);

  /* I2C1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMA_REQUEST_3);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00D09BE3;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**I2C2 GPIO Configuration
  PB10   ------> I2C2_SCL
  PB11   ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* I2C2 DMA Init */

  /* I2C2_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_3);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* I2C2_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_3);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  /* I2C2 interrupt Init */
  NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(I2C2_ER_IRQn);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableAutoEndMode(I2C2);
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00D09BE3;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 80;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOH);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(RED_GPIO_Port, RED_Pin);

  /**/
  GPIO_InitStruct.Pin = RED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
