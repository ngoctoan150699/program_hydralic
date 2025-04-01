/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CanBus.h"
#include "MCP4922.h"
#include "hydraulic.h"
#include "MotorControl.h"
#include "MCP3202.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static CAN_COM canOpen ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN_ADC 0
#define MAX_ADC 2000
#define MAX_FLOAT 1.0f
#define AUTO_TIME 20000 // 20 giây (đơn vị: ms)
#define STOP_TIME 5000
#define TIME_FREE 20000
#define TIME_LIFT_PALLET 4000
#define TIME_LOWER_WHEEL 6000
#define TIME_DOWN_PALLET 2000
#define TIME_LIFT_WHEEL 2000
volatile uint8_t buttonPressCount = 0;
volatile bool modeLift = false;
volatile uint8_t autoStep = 0;
volatile uint8_t autoStep_motor = 0;
volatile uint32_t totalCycle = 0;
bool stepDone[4] = {false};
uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 50;
volatile uint8_t lastButtonState = 0;
static bool mode = false;
static bool reset_motor = false;
uint64_t timer_hydarulic[2] = {0};
// Mảng global lưu trạng thái các GPIO
uint8_t gpioInputStates[5] = {0};  // 5 phần tử tương ứng với A6, F10, F7, F9, C2
int a;
uint16_t adc_speed;
static double target_speed = 0;
static bool forward_pressed=0;
static bool reverse_pressed=0;
static int dir = 0;
#define OperationMode    0x3U
#define ControlWord_EN   0x0FU
#define ControlWord_DIS  0x06U
#define OperationModeRes 0x60600008U
#define ControlWordRes   0x60400010U
#define TargetSpeedRes   0x60FF0020U
#define ProfileAccRes    0x60830020U
#define ProfileDecRes    0x60840020U
#define ResetErrorsRes   0x20100210U
#define MNum 2
uint8_t TX_ENABLE[8]={0x2B, 0x40 ,0x60 ,0x00 ,0x0F ,0x00 ,0x00 ,0x00};
uint8_t TX_DISENABLE[8]={0x2B, 0x40 ,0x60 ,0x00 ,0x06 ,0x00 ,0x00 ,0x00};
//static char buffer_mes[12];
const struct HydraulicTableControl wheel_up_state     = {0, 0, 1, 1} ;
const struct HydraulicTableControl wheel_down_state   = {1, 0, 1, 1} ;
const struct HydraulicTableControl pallet_up_state    = {1, 1, 0, 1} ;
const struct HydraulicTableControl pallet_down_state  = {0, 1, 0, 1} ;
const struct HydraulicTableControl free_all_state     = {0, 0, 0, 0} ;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Pump */
osThreadId_t Task_PumpHandle;
const osThreadAttr_t Task_Pump_attributes = {
  .name = "Task_Pump",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Task_Motor */
osThreadId_t Task_MotorHandle;
const osThreadAttr_t Task_Motor_attributes = {
  .name = "Task_Motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void StartTask_Pump(void *argument);
void StartTask_Motor(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool u_timer_expired(uint64_t *t, uint64_t prd, uint64_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}
//
//void manualMode(void) {
//    modeLift = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0) == GPIO_PIN_SET ? true : false; // TRUE: bánh xe, FALSE: pallet
//
//    uint16_t checkPin = modeLift ? GPIO_PIN_11 : GPIO_PIN_12; // Chọn pin theo chế độ
//    int reading = HAL_GPIO_ReadPin(GPIOF, checkPin);
//
//    if (reading != lastButtonState) {
//        lastDebounceTime = HAL_GetTick();
//    }
//
//    if ((HAL_GetTick() - lastDebounceTime) > debounceDelay) {
//        if (reading != buttonPressCount) {
//            lastButtonState = reading;
//            if (reading == GPIO_PIN_RESET) { // Nút nhấn
//                buttonPressCount++;
//                if (buttonPressCount > 4) buttonPressCount = 1;
//            }
//        }
//    }
//}

void readGPIOInputs(void) {
    gpioInputStates[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
    gpioInputStates[1] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10);
    gpioInputStates[2] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7);
    gpioInputStates[3] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6);
    gpioInputStates[4] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8);
}
void manualMode(void) {
    modeLift = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET ? true : false; // TRUE: bánh xe, FALSE: pallet

    bool liftPressed = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7) == GPIO_PIN_RESET ? true : false;
    bool lowerPressed = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9) == GPIO_PIN_RESET ? true : false;

    if (liftPressed) {
        buttonPressCount = 1;  // Nâng
    }
    else if (lowerPressed) {
        buttonPressCount = 3;  // Hạ
    }
    else {
        buttonPressCount = 0;  // Tắt thủy lực
    }
}

void autoMode()
{
  switch (autoStep)
  {
  case 0: // nâng pallet
    buttonPressCount = 1;
    modeLift = false;
    if (u_timer_expired(&timer_hydarulic[0], TIME_LIFT_PALLET, HAL_GetTick()))
    {
      stepDone[0] = true;
    }
    if (stepDone[0])
    {
      buttonPressCount = 0; // off thuỷ lực
      if (u_timer_expired(&timer_hydarulic[1], TIME_FREE, HAL_GetTick()))
      {
        stepDone[0] = false;
        memset(timer_hydarulic, 0, sizeof(timer_hydarulic));
        autoStep++;
      }
    }
    break;

  case 1: // hạ bánh xe
    buttonPressCount = 1;
    modeLift = true;
    if (u_timer_expired(&timer_hydarulic[0], TIME_LOWER_WHEEL, HAL_GetTick()))
    {
      stepDone[1] = true;
    }
    if (stepDone[1])
    {
      buttonPressCount = 0; // off thuỷ lực
      if (u_timer_expired(&timer_hydarulic[1], TIME_FREE, HAL_GetTick()))
      {
        stepDone[1] = false;
        memset(timer_hydarulic, 0, sizeof(timer_hydarulic));
        autoStep++;
      }
    }
    break;

  case 2: // hạ pallet
    buttonPressCount = 3;
    modeLift = false;
    if (u_timer_expired(&timer_hydarulic[0], TIME_DOWN_PALLET, HAL_GetTick()))
    {
      stepDone[2] = true;
    }
    if (stepDone[2])
    {
      buttonPressCount = 0; // off thuỷ lực
      if (u_timer_expired(&timer_hydarulic[1], TIME_FREE, HAL_GetTick()))
      {
        stepDone[2] = false;
        memset(timer_hydarulic, 0, sizeof(timer_hydarulic));
        autoStep++;
      }
    }
    break;

  case 3: // nâng bánh xe
    buttonPressCount = 3;
    modeLift = true;
    if (u_timer_expired(&timer_hydarulic[0], TIME_LIFT_WHEEL, HAL_GetTick()))
    {
      stepDone[3] = true;
    }
    if (stepDone[3])
    {
      buttonPressCount = 0; // off thuỷ lực
      if (u_timer_expired(&timer_hydarulic[1], TIME_FREE, HAL_GetTick()))
      {
        stepDone[3] = false;
        memset(timer_hydarulic, 0, sizeof(timer_hydarulic));
        autoStep = 0;
        totalCycle++;
      }
    }
    break;

  default:
    break;
  }
}

void performAction(int count, bool Mode) {
	switch (count) {
	case 1:
		if (Mode) {
			hydraulicSetState(wheel_down_state);
			// Thực hiện hành động hạ bánh xe
		} else {
			hydraulicSetState(pallet_up_state);
			// Thực hiện hành động nâng pallet
		}
		break;
	case 3:
		if (Mode) {
			hydraulicSetState(wheel_up_state);
			// Thực hiện hành động nâng bánh xe
		} else {
			hydraulicSetState(pallet_down_state);
			// Thực hiện hành động hạ pallet
		}
		break;
	default:
		hydraulicSetState(free_all_state);
		// Trạng thái tự do
		break;
	}
}

//void canOpenCallBack() {
//	a++;
//	static uint32_t canId;
//	canId = canOpen.Can_rxHeader.StdId;
//	memcpy(buffer_mes, canOpen.Can_rxData, 8);
//	if ((canId >= 0x018A && canId <= 0x018E) || canId == 0x70A) {
//
//	} else {
//		CanRecieverCallback(); // hàm nhận dữ liệu của động cơ
//	}
//}

float map_adc_to_float(uint16_t adc) {
    if (adc < MIN_ADC) adc = MIN_ADC;
    if (adc > MAX_ADC) adc = MAX_ADC;
    return ((adc - MIN_ADC) / (float)(MAX_ADC - MIN_ADC)) * MAX_FLOAT;
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
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  mcp4922.begin(&hspi1,GPIOA,GPIO_PIN_4);
  mcp3202.begin(&hspi2,GPIOB,GPIO_PIN_9);

//  Can_begin(&canOpen, &hcan2, 0);
//  CanCofigfilter(&canOpen, 0x11, 0x11);
//  canOpen.CanRxIT_Callback = &canOpenCallBack;
//  Can_Start(&canOpen, MotorID[0]);
  MotorInit(&hcan2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_Pump */
  Task_PumpHandle = osThreadNew(StartTask_Pump, NULL, &Task_Pump_attributes);

  /* creation of Task_Motor */
  Task_MotorHandle = osThreadNew(StartTask_Motor, NULL, &Task_Motor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
 //CanCofigfilter(&CanUser1,FilterHigh, FilterLOw);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 21;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF6 PF7 PF8 PF9
                           PF10 PF11 PF12 PF13
                           PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG4 PG5 PG6 PG7
                           PG10 PG11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask_Pump */
/**
* @brief Function implementing the Task_Pump thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Pump */
void StartTask_Pump(void *argument)
{
  /* USER CODE BEGIN StartTask_Pump */
  /* Infinite loop */
  for (;;)
  {
	  readGPIOInputs();
      // Kiểm tra nút RESET (GPIOG_PIN_2)
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
      {
    	 // motorErrorReset();
    	  NMTmanagement(0x2, MotorID[0]);
          hydraulicSetState(free_all_state); // Dừng mọi hoạt động
          // Reset tất cả
          buttonPressCount = 0;
          autoStep = 0;
          modeLift = false;
          memset(timer_hydarulic, 0, sizeof(timer_hydarulic));  // Reset timer
          memset(stepDone, 0, sizeof(stepDone));  // Reset cờ chạy auto
          continue;
      }



      mode = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10) == GPIO_PIN_SET ? true : false;

      if (mode) {
          autoMode();
      } else {
          manualMode();
          memset(timer_hydarulic, 0, sizeof(timer_hydarulic));  // Reset timer chế độ auto
          memset(stepDone, 0, sizeof(stepDone));  // Reset cờ chạy auto
      }

      performAction(buttonPressCount, modeLift);
      osDelay(10);
  }
  /* USER CODE END StartTask_Pump */
}

/* USER CODE BEGIN Header_StartTask_Motor */
/**
* @brief Function implementing the Task_Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Motor */
void StartTask_Motor(void *argument) {
    /* USER CODE BEGIN StartTask_Motor */
    osDelay(5000);
 //   NMTmanagement(0x2, MotorID[0]);
    SetOperationMode(3, MotorID[0]);
    SDOProfileAcc(speedToRps(0.25), MotorID[0]);
    SDOProfileDec(speedToRps(0.3), MotorID[0]);
    SetControlWord(ControlWord_EN, MotorID[0]); // enable motor
    NMTmanagement(0x1, MotorID[0]);

    static uint32_t last_change_time = 0;
    static bool stopping_phase = false;
    static int last_dir = -1;
    static float last_target_speed = -1;

    /* Infinite loop */
    for (;;) {
        reset_motor = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET;
        adc_speed = mcp3202.readChannel(0);
        bool m_error = false;

        if (reset_motor == 0) {
            if (mode == 0) { // Chế độ Manual
                forward_pressed = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) == GPIO_PIN_SET);
                reverse_pressed = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8) == GPIO_PIN_SET);

                if (forward_pressed && reverse_pressed) {
                    dir = 0;
                    target_speed = 0;
                } else if (forward_pressed) {
                    dir = 1;
                    target_speed = map_adc_to_float(adc_speed);
                } else if (reverse_pressed) {
                    dir = 3;
                    target_speed = map_adc_to_float(adc_speed);
                } else {
                    dir = 0;
                    target_speed = 0;
                }

            } else { // Chế độ Auto
                uint32_t current_time = HAL_GetTick();

                if (stopping_phase) {
                    if (current_time - last_change_time >= STOP_TIME) {
                        stopping_phase = false;
                        last_change_time = current_time;
                        dir = (dir == 1) ? 3 : 1;
                    } else {
                        dir = 0;
                    }
                } else {
                    if (current_time - last_change_time >= AUTO_TIME) {
                        stopping_phase = true;
                        last_change_time = current_time;
                        dir = 0;
                    }
                }
            }
            target_speed = map_adc_to_float(adc_speed);

            if (dir != last_dir || target_speed != last_target_speed) {
                motorControl(true, m_error, dir, target_speed);
                last_dir = dir;
                last_target_speed = target_speed;
            }
        } else {
            if (last_dir != 0 || last_target_speed != 0) {
                NMTmanagement(0x1, MotorID[0]);
                motorControl(true, m_error, 0, 0);
                last_dir = 0;
                last_target_speed = 0;
            }
        }
        osDelay(50);
    }
    /* USER CODE END StartTask_Motor */
}


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
