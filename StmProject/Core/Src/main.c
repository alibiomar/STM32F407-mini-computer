/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F407E-DISCO with OLED Display and HID Keyboard
  *                   Button Configuration (4-button control):
  *                   PC1 = Exit button (returns to home/exits screens)
  *                   PC2 = Confirm/OK button (selects/confirms actions)
  *                   PC3 = Up/Previous button (navigates backward)
  *                   PC4 = Down/Next/Exit button (exits screens/games)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "usbh_hid.h"
#include "usbh_hid_keybd.h"
#include "game.h"
#include "fonts.h"
#include "SH1106.h"
#include "bitmap.h"
#include "bootAnim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Button and LED definitions moved to main.h

#define MAX_TEXT_LENGTH 2000
#define DISPLAY_LINES 8
#define DISPLAY_CHARS_PER_LINE 21

#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
// Simplified - only essential mutexes
SemaphoreHandle_t dataMutexHandle;      // Protects all shared data
SemaphoreHandle_t displayUpdateFlag;     // Signals display needs update

TaskHandle_t mainLoopTaskHandle;
TaskHandle_t uartTaskHandle;

// Button variables
GPIO_PinState lastExitState = GPIO_PIN_SET;
GPIO_PinState lastConfirmState = GPIO_PIN_SET;
GPIO_PinState lastUpState = GPIO_PIN_SET;
GPIO_PinState lastDownState = GPIO_PIN_SET;
uint32_t lastExitDebounceTime = 0;
uint32_t lastConfirmDebounceTime = 0;
uint32_t lastUpDebounceTime = 0;
uint32_t lastDownDebounceTime = 0;
#define DEBOUNCE_DELAY 50


// Display buffer
typedef struct {
  char text[MAX_TEXT_LENGTH];
  uint16_t length;
  bool needsUpdate;
} TextBuffer_t;

TextBuffer_t textBuffer = {0};

// Email info to display
typedef struct {
  char recipient[64];
  char status[32];
  bool hasRecipient;
  bool textConfirmed;
} EmailInfo_t;

EmailInfo_t emailInfo = {0};

char ipEsp[20] = {0};
bool displayNeedsUpdate = false;

#define DESKTOP_ITEM_COUNT 5
uint8_t selectedDesktopItem = 0;

// LED state tracking
typedef struct {
  bool green;
  bool orange;
  bool red;
  bool blue;
} LEDStates_t;

LEDStates_t ledStates = {false, false, false, false};

// Keyboard data
extern USBH_HandleTypeDef hUsbHostFS;
HID_KEYBD_Info_TypeDef *keyboardInfo;
uint8_t lastKeyState[6] = {0};
bool keyboardConnected = false;

// Display mode
typedef enum {
  DISPLAY_MODE_HOME,
  DISPLAY_MODE_TEXT,
  DISPLAY_MODE_EMAIL_SETUP,
  DISPLAY_MODE_EMAIL_STATUS,
  DISPLAY_MODE_SYSTEM_INFO,
  DISPLAY_MODE_CALCULATOR,
  DISPLAY_MODE_GAME,
  DISPLAY_MODE_SETTINGS
} DisplayMode_t;

DisplayMode_t displayMode = DISPLAY_MODE_HOME;
typedef struct {
    const char* name;
    const uint8_t* icon;
    DisplayMode_t targetMode;
} DesktopItem_t;

// Desktop menu items
DesktopItem_t desktopItems[] = {
    {"Email", icon_email, DISPLAY_MODE_TEXT},
	{"Calc", icon_calculator, DISPLAY_MODE_CALCULATOR},
	 {"Games", icon_game, DISPLAY_MODE_GAME},
	 {"Settings", icon_system, DISPLAY_MODE_SETTINGS},
	    {"System", icon_system, DISPLAY_MODE_SYSTEM_INFO},

};
static int8_t carouselPosition = 0;
// Email mode for keyboard input
static bool emailMode = false;
static char emailBuffer[64];
static uint8_t emailIdx = 0;

// UART RX interrupt buffer
#define RX_BUFFER_SIZE 256
uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxHead = 0;
volatile uint16_t rxTail = 0;
volatile bool rxOverflow = false;
uint8_t uartRxByte;  // Single byte for interrupt reception
// Calculator variables
static char calcDisplay[22] = "0";
static char calcBuffer[64] = "";
static char lastOperator = 0;
static double lastValue = 0.0;
static bool newNumber = true;
static bool errorState = false;

typedef struct {
  char ssid[MAX_SSID_LENGTH + 1];
  char password[MAX_PASSWORD_LENGTH + 1];
  uint8_t ssidIdx;
  uint8_t passwordIdx;
  bool editingSSID;  // true = editing SSID, false = editing password
  char status[32];
} WiFiSettings_t;

WiFiSettings_t wifiSettings = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void MainLoopTask(void *argument);
void UARTReceiveTask(void *argument);
void Initialize_Resources(void);
void ProcessKeyboard(void);
void ProcessButtons(void);
void RequestDisplayUpdate(void);
char ConvertHIDKeyToChar(uint8_t keyCode, bool shift);
void UpdateDisplay(void);
void DisplayHomeScreen(void);
void DisplayTextScreen(void);
void DisplayEmailSetupScreen(void);
void DisplayEmailStatusScreen(void);
void AddCharToBuffer(char c);
void ClearTextBuffer(void);
void DisplaySystemInfoScreen(void);
void HandleConfirmButton(void);
void HandleExitButton(void);
void HandleUpButton(void);
void HandleDownButton(void);
void SendEmailCommand(void);
void SendCharToESP(char c);
void DisplayCalculatorScreen(void);
void HandleCalculatorInput(char c);
void CalculatorClear(void);
void CalculatorBackspace(void);
void CalculatorExecute(void);
void DisplaySettingsScreen(void);
void HandleSettingsInput(char c);
void SendWiFiConfigToESP(void);

// FreeRTOS tasks (forward declarations)
void DisplayTask(void *argument);
void GameUpdateTask(void *argument);
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
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  Initialize_Resources();

  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  // Create all necessary tasks
  xTaskCreate(MainLoopTask, "mainLoop", 1024, NULL, 2, &mainLoopTaskHandle);
  xTaskCreate(UARTReceiveTask, "uartRecv", 512, NULL, 1, &uartTaskHandle);
  xTaskCreate(DisplayTask, "display", 512, NULL, 2, NULL);
  xTaskCreate(GameUpdateTask, "gameUpdate", 512, NULL, 2, NULL);
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);
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

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|GPIO_PIN_13|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 PC3 PC4 - All button inputs */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // Enable pull-up resistors for buttons
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin PD13 LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|GPIO_PIN_13|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Initialize_Resources(void) {
  dataMutexHandle = xSemaphoreCreateMutex();
  if (dataMutexHandle == NULL) {
    Error_Handler();
  }

  displayUpdateFlag = xSemaphoreCreateBinary();
  if (displayUpdateFlag == NULL) {
    Error_Handler();
  }

  // *** ADD THIS ***
  strcpy(ipEsp, "");
  strcpy(emailInfo.status, "Ready");
}

// Request display update from any context
void RequestDisplayUpdate(void) {
  xSemaphoreGive(displayUpdateFlag);
}

// SIMPLIFIED: Single main loop task handles keyboard, buttons, and display
void MainLoopTask(void *argument) {
  vTaskDelay(pdMS_TO_TICKS(500));

  for (;;) {
    // 1. Process USB keyboard
    ProcessKeyboard();

    // 2. Process buttons
    ProcessButtons();

    // 3. Check if display needs update
    if (xSemaphoreTake(displayUpdateFlag, 0) == pdTRUE) {
      // IMPORTANT: Display rendering must be single-owner.
      // DisplayTask handles all OLED updates (including game rendering).
      // Calling UpdateDisplay() here can clear the screen while a game is rendering.
      if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(20)) == pdTRUE) {
        displayNeedsUpdate = true;
        xSemaphoreGive(dataMutexHandle);
      } else {
        // Best-effort: even without mutex, flagging an update is safe for a bool.
        displayNeedsUpdate = true;
      }
    }

    // 4. Give other tasks time
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void ProcessKeyboard(void) {
  if (USBH_HID_GetDeviceType(&hUsbHostFS) != HID_KEYBOARD) {
    if (keyboardConnected) {
      if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        keyboardConnected = false;
        memset(lastKeyState, 0, 6);
        xSemaphoreGive(dataMutexHandle);
      }
    }
    return;
  }

  keyboardInfo = USBH_HID_GetKeybdInfo(&hUsbHostFS);
  if (keyboardInfo == NULL) return;

  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(50)) != pdTRUE) {
    return;
  }

  if (!keyboardConnected) {
    keyboardConnected = true;
    HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
  }

  bool currentShiftState = (keyboardInfo->lshift || keyboardInfo->rshift);

  for (uint8_t i = 0; i < 6; i++) {
    if (keyboardInfo->keys[i] != 0) {
      bool keyFound = false;
      for (uint8_t j = 0; j < 6; j++) {
        if (keyboardInfo->keys[i] == lastKeyState[j]) {
          keyFound = true;
          break;
        }
      }

      if (!keyFound) {
        uint8_t keyCode = keyboardInfo->keys[i];
        char c = 0;

        if (keyCode == 0x28) {
          c = '\n';
        }
        else if (keyCode == 0x2A) {
          if (emailMode) {
            if (emailIdx > 0) {
              emailIdx--;
              emailBuffer[emailIdx] = '\0';
            }
          } else {
            if (textBuffer.length > 0) {
              textBuffer.length--;
              textBuffer.text[textBuffer.length] = '\0';
            }
          }
          displayNeedsUpdate = true;
          HAL_GPIO_TogglePin(LED_PORT, LED_RED_PIN);
          xSemaphoreGive(dataMutexHandle);
          RequestDisplayUpdate();
          memcpy(lastKeyState, keyboardInfo->keys, 6);
          return;
        }
        else if (keyCode == 0x2C) {
          c = ' ';
        }
        else {
          c = ConvertHIDKeyToChar(keyCode, currentShiftState);
        }

        if (c != 0) {
          if (emailMode) {
            if (emailIdx < 63) {
              emailBuffer[emailIdx++] = c;
            }
          } else {
            if (textBuffer.length < MAX_TEXT_LENGTH - 1) {
              textBuffer.text[textBuffer.length++] = c;
              textBuffer.text[textBuffer.length] = '\0';
            }

            char keyMsg[15];
            snprintf(keyMsg, sizeof(keyMsg), "KEY:%c\n", c);
            HAL_UART_Transmit(&huart6, (uint8_t*)keyMsg, strlen(keyMsg), 100);
          }

          displayNeedsUpdate = true;
          HAL_GPIO_TogglePin(LED_PORT, LED_BLUE_PIN);
        }
      }
    }
  }

  memcpy(lastKeyState, keyboardInfo->keys, 6);
  xSemaphoreGive(dataMutexHandle);

  if (displayNeedsUpdate) {
    RequestDisplayUpdate();
  }
}

void ProcessButtons(void) {
  GPIO_PinState currentExitState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_EXIT_PIN);
  GPIO_PinState currentConfirmState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_CONFIRM_PIN);
  GPIO_PinState currentUpState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_UP_PIN);
  GPIO_PinState currentDownState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_DOWN_PIN);
  uint32_t currentTime = HAL_GetTick();

  if (currentExitState == GPIO_PIN_RESET && lastExitState == GPIO_PIN_SET) {
    if ((currentTime - lastExitDebounceTime) > DEBOUNCE_DELAY) {
      lastExitDebounceTime = currentTime;
      HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);
      HandleExitButton();
      vTaskDelay(pdMS_TO_TICKS(50));
      HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
    }
  }

  if (currentConfirmState == GPIO_PIN_RESET && lastConfirmState == GPIO_PIN_SET) {
    if ((currentTime - lastConfirmDebounceTime) > DEBOUNCE_DELAY) {
      lastConfirmDebounceTime = currentTime;
      HAL_GPIO_WritePin(LED_PORT, LED_BLUE_PIN, GPIO_PIN_SET);
      HandleConfirmButton();
      vTaskDelay(pdMS_TO_TICKS(50));
      HAL_GPIO_WritePin(LED_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
    }
  }

  if (currentUpState == GPIO_PIN_RESET && lastUpState == GPIO_PIN_SET) {
    if ((currentTime - lastUpDebounceTime) > DEBOUNCE_DELAY) {
      lastUpDebounceTime = currentTime;
      HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
      HandleUpButton();
      vTaskDelay(pdMS_TO_TICKS(50));
      HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
    }
  }

  if (currentDownState == GPIO_PIN_RESET && lastDownState == GPIO_PIN_SET) {
    if ((currentTime - lastDownDebounceTime) > DEBOUNCE_DELAY) {
      lastDownDebounceTime = currentTime;
      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      HandleDownButton();
      vTaskDelay(pdMS_TO_TICKS(50));
      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
    }
  }

  lastExitState = currentExitState;
  lastConfirmState = currentConfirmState;
  lastUpState = currentUpState;
  lastDownState = currentDownState;
}

/* ============================================================================
   UART Receive Complete Callback - ISR Context
   ============================================================================ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART6) {
    // Calculate next head position
    uint16_t nextHead = (rxHead + 1) % RX_BUFFER_SIZE;

    // Check for buffer overflow
    if (nextHead == rxTail) {
      rxOverflow = true;
      // Drop this byte, but continue receiving
    } else {
      // Store byte in circular buffer
      rxBuffer[rxHead] = uartRxByte;
      rxHead = nextHead;
    }

    // Re-enable interrupt for next byte
    HAL_UART_Receive_IT(&huart6, &uartRxByte, 1);
  }
}

/* ============================================================================
   Helper function to read from circular buffer - Call from task context
   ============================================================================ */
bool UART_ReadByte(uint8_t *byte) {
  // Disable interrupts briefly to read head position safely
  __disable_irq();

  if (rxHead == rxTail) {
    __enable_irq();
    return false;  // Buffer empty
  }

  // Read byte
  *byte = rxBuffer[rxTail];
  rxTail = (rxTail + 1) % RX_BUFFER_SIZE;

  __enable_irq();
  return true;
}

void CalculatorClear(void) {
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        strcpy(calcDisplay, "0");
        memset(calcBuffer, 0, sizeof(calcBuffer));
        lastOperator = 0;
        lastValue = 0.0;
        newNumber = true;
        errorState = false;
        textBuffer.needsUpdate = true;
        xSemaphoreGive(dataMutexHandle);
    }
}

void CalculatorBackspace(void) {
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t len = strlen(calcDisplay);
        if (len > 1 && !errorState) {
            calcDisplay[len - 1] = '\0';
        } else {
            strcpy(calcDisplay, "0");
        }
        textBuffer.needsUpdate = true;
        xSemaphoreGive(dataMutexHandle);
    }
}

void CalculatorExecute(void) {
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (errorState) {
            xSemaphoreGive(dataMutexHandle);
            return;
        }

        double currentValue = atof(calcDisplay);
        double result = currentValue;

        if (lastOperator != 0) {
            switch(lastOperator) {
                case '+': result = lastValue + currentValue; break;
                case '-': result = lastValue - currentValue; break;
                case '*': result = lastValue * currentValue; break;
                case '/':
                    if (currentValue != 0.0) {
                        result = lastValue / currentValue;
                    } else {
                        strcpy(calcDisplay, "Error");
                        errorState = true;
                        textBuffer.needsUpdate = true;
                        xSemaphoreGive(dataMutexHandle);
                        return;
                    }
                    break;
            }
        }

        // Format result
        if (result > 999999999.0 || result < -99999999.0) {
            strcpy(calcDisplay, "Overflow");
            errorState = true;
        } else if (result == (int)result) {
            snprintf(calcDisplay, sizeof(calcDisplay), "%d", (int)result);
        } else {
            snprintf(calcDisplay, sizeof(calcDisplay), "%.4f", result); // @suppress("Float formatting support")
            // Trim trailing zeros
            uint8_t len = strlen(calcDisplay);
            while (len > 0 && calcDisplay[len-1] == '0') {
                calcDisplay[--len] = '\0';
            }
            if (len > 0 && calcDisplay[len-1] == '.') {
                calcDisplay[--len] = '\0';
            }
        }

        lastOperator = 0;
        lastValue = 0.0;
        newNumber = true;
        textBuffer.needsUpdate = true;
        xSemaphoreGive(dataMutexHandle);
    }
}

void HandleCalculatorInput(char c) {
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (errorState && c != 'C' && c != 'c') {
            xSemaphoreGive(dataMutexHandle);
            return;
        }

        // Clear on C
        if (c == 'C' || c == 'c') {
            strcpy(calcDisplay, "0");
            memset(calcBuffer, 0, sizeof(calcBuffer));
            lastOperator = 0;
            lastValue = 0.0;
            newNumber = true;
            errorState = false;
            textBuffer.needsUpdate = true;
            xSemaphoreGive(dataMutexHandle);
            return;
        }

        // Handle digits and decimal point
        if ((c >= '0' && c <= '9') || c == '.') {
            if (newNumber) {
                if (c == '.') {
                    strcpy(calcDisplay, "0.");
                } else {
                    calcDisplay[0] = c;
                    calcDisplay[1] = '\0';
                }
                newNumber = false;
            } else {
                uint8_t len = strlen(calcDisplay);
                if (len < 21) {
                    if (c == '.' && strchr(calcDisplay, '.') != NULL) {
                        // Already has decimal point
                    } else {
                        calcDisplay[len] = c;
                        calcDisplay[len + 1] = '\0';
                    }
                }
            }
            textBuffer.needsUpdate = true;
        }
        // Handle operators
        else if (c == '+' || c == '-' || c == '*' || c == '/') {
            if (lastOperator != 0) {
                // Execute previous operation first
                double currentValue = atof(calcDisplay);
                double result = currentValue;

                switch(lastOperator) {
                    case '+': result = lastValue + currentValue; break;
                    case '-': result = lastValue - currentValue; break;
                    case '*': result = lastValue * currentValue; break;
                    case '/':
                        if (currentValue != 0.0) {
                            result = lastValue / currentValue;
                        } else {
                            strcpy(calcDisplay, "Error");
                            errorState = true;
                            textBuffer.needsUpdate = true;
                            xSemaphoreGive(dataMutexHandle);
                            return;
                        }
                        break;
                }

                snprintf(calcDisplay, sizeof(calcDisplay), "%.4f", result); // @suppress("Float formatting support")
                lastValue = result;
            } else {
                lastValue = atof(calcDisplay);
            }

            lastOperator = c;
            newNumber = true;
            textBuffer.needsUpdate = true;
        }

        xSemaphoreGive(dataMutexHandle);
    }
}

void GameUpdateTask(void *argument) {
  for(;;) {
    if (gameMode) {
      Game_Update();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void ButtonTask(void *argument) {
  GPIO_PinState currentExitState;
  GPIO_PinState currentConfirmState;
  GPIO_PinState currentUpState;
  GPIO_PinState currentDownState;

  // Initial state reading
  lastExitState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_EXIT_PIN);
  lastConfirmState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_CONFIRM_PIN);
  lastUpState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_UP_PIN);
  lastDownState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_DOWN_PIN);

  vTaskDelay(pdMS_TO_TICKS(100)); // Initial stabilization delay

  for(;;) {
    // Read button states
    currentExitState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_EXIT_PIN);
    currentConfirmState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_CONFIRM_PIN);
    currentUpState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_UP_PIN);
    currentDownState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_DOWN_PIN);

    // Handle Exit Button (PC1) - Return to home/exit screens
    // Button pressed: HIGH to LOW transition (pull-up resistor)
    if (currentExitState == GPIO_PIN_RESET && lastExitState == GPIO_PIN_SET) {
      uint32_t currentTime = HAL_GetTick();
      if ((currentTime - lastExitDebounceTime) > DEBOUNCE_DELAY) {
        lastExitDebounceTime = currentTime;

        // Visual feedback - turn ON LED
        HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);

        // Handle button action
        HandleExitButton();

        // Keep LED on briefly for visual feedback
        vTaskDelay(pdMS_TO_TICKS(100));
        HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
      }
    }

    // Handle Confirm Button (PC2) - Action/OK
    if (currentConfirmState == GPIO_PIN_RESET && lastConfirmState == GPIO_PIN_SET) {
      uint32_t currentTime = HAL_GetTick();
      if ((currentTime - lastConfirmDebounceTime) > DEBOUNCE_DELAY) {
        lastConfirmDebounceTime = currentTime;

        // Visual feedback - turn ON LED
        HAL_GPIO_WritePin(LED_PORT, LED_BLUE_PIN, GPIO_PIN_SET);

        // Handle button action
        HandleConfirmButton();

        // Keep LED on briefly for visual feedback
        vTaskDelay(pdMS_TO_TICKS(100));
        HAL_GPIO_WritePin(LED_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
      }
    }

    // Handle Up Button (PC3) - Navigate Up/Previous
    if (currentUpState == GPIO_PIN_RESET && lastUpState == GPIO_PIN_SET) {
      uint32_t currentTime = HAL_GetTick();
      if ((currentTime - lastUpDebounceTime) > DEBOUNCE_DELAY) {
        lastUpDebounceTime = currentTime;

        // Visual feedback - turn ON LED
        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);

        // Handle button action
        HandleUpButton();

        // Keep LED on briefly for visual feedback
        vTaskDelay(pdMS_TO_TICKS(100));
        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
      }
    }

    // Handle Down Button (PC4) - Navigate Down/Next/Exit
    if (currentDownState == GPIO_PIN_RESET && lastDownState == GPIO_PIN_SET) {
      uint32_t currentTime = HAL_GetTick();
      if ((currentTime - lastDownDebounceTime) > DEBOUNCE_DELAY) {
        lastDownDebounceTime = currentTime;

        // Visual feedback - turn ON LED
        HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);

        // Handle button action
        HandleDownButton();

        // Keep LED on briefly for visual feedback
        vTaskDelay(pdMS_TO_TICKS(100));
        HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
      }
    }

    lastExitState = currentExitState;
    lastConfirmState = currentConfirmState;
    lastUpState = currentUpState;
    lastDownState = currentDownState;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void HandleExitButton(void) {
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (gameMode) {
            // In game mode - Exit button
            Game_HandleInput(1); // 1 = Exit button
        } else {
            // Exit button - return to home from any screen
            if (displayMode != DISPLAY_MODE_HOME) {
                displayMode = DISPLAY_MODE_HOME;
                emailMode = false;
                emailIdx = 0;
                memset(emailBuffer, 0, sizeof(emailBuffer));
                displayNeedsUpdate = true;
            }
        }
        xSemaphoreGive(dataMutexHandle);
        RequestDisplayUpdate();
    }
}

void HandleConfirmButton(void) {
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    DisplayMode_t currentMode = displayMode;

    switch(currentMode) {
        case DISPLAY_MODE_HOME:
            // Select the current carousel item
            if (carouselPosition >= 0 && carouselPosition < DESKTOP_ITEM_COUNT) {
                displayMode = desktopItems[carouselPosition].targetMode;
                if (displayMode == DISPLAY_MODE_TEXT) {
                    emailMode = false;
                    memset(textBuffer.text, 0, sizeof(textBuffer.text));
                    textBuffer.length = 0;
                } else if (displayMode == DISPLAY_MODE_GAME) {
                    Game_Init();
                    // Don't set gameMode = true here! Let Game_Start() do it when user selects a game
                    textBuffer.needsUpdate = true; // Ensure display updates
                }
                displayNeedsUpdate = true;
            }
            break;

        case DISPLAY_MODE_TEXT:
            if (textBuffer.length > 0) {
                displayMode = DISPLAY_MODE_EMAIL_SETUP;
                emailMode = true;
                emailIdx = 0;
                memset(emailBuffer, 0, sizeof(emailBuffer));
                displayNeedsUpdate = true;
            }
            break;

        case DISPLAY_MODE_EMAIL_SETUP:
            if (emailIdx > 0) {
                emailBuffer[emailIdx < 63 ? emailIdx : 63] = '\0';
                strncpy(emailInfo.recipient, emailBuffer, 63);
                emailInfo.recipient[63] = '\0';
                emailMode = false;
                strcpy(emailInfo.status, "Sending...");
                displayMode = DISPLAY_MODE_EMAIL_STATUS;
                displayNeedsUpdate = true;
                xSemaphoreGive(dataMutexHandle);
                RequestDisplayUpdate();
                vTaskDelay(pdMS_TO_TICKS(50));
                SendEmailCommand();
                return;
            }
            break;

        case DISPLAY_MODE_SYSTEM_INFO:
            xSemaphoreGive(dataMutexHandle);
            const char* refreshMsg = "TEST:REQUEST_IP\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)refreshMsg, strlen(refreshMsg), 100);
            HAL_GPIO_TogglePin(LED_PORT, LED_GREEN_PIN);
            return;

        case DISPLAY_MODE_EMAIL_STATUS:
            displayMode = DISPLAY_MODE_HOME;
            memset(textBuffer.text, 0, sizeof(textBuffer.text));
            textBuffer.length = 0;
            displayNeedsUpdate = true;
            HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
            break;
            
        case DISPLAY_MODE_CALCULATOR:
            // Calculator-specific action (could be equals, evaluate, etc.)
            // For now, do nothing - let PC3 clear and PC4 exit
            break;

        case DISPLAY_MODE_SETTINGS:
            // Settings-specific action if needed
            break;

        case DISPLAY_MODE_GAME:
            // Game handles its own input (both menu and gameplay)
            Game_HandleInput(2);
            break;
    }

    xSemaphoreGive(dataMutexHandle);
    RequestDisplayUpdate();
}

void HandleUpButton(void) {
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (displayMode == DISPLAY_MODE_GAME) {
      // In game display mode - Up button for game control (menu & gameplay)
      Game_HandleInput(3); // 3 = Up button
        } else {
            // Right button - Navigate right or perform secondary actions
            switch(displayMode) {
                case DISPLAY_MODE_HOME:
                    // Navigate right (next item)
                    carouselPosition = (carouselPosition + 1) % DESKTOP_ITEM_COUNT;
                    selectedDesktopItem = carouselPosition;
                    displayNeedsUpdate = true;
                    break;

                case DISPLAY_MODE_TEXT:
                    // Clear text buffer
                    memset(textBuffer.text, 0, sizeof(textBuffer.text));
                    textBuffer.length = 0;
                    displayNeedsUpdate = true;
                    break;

                case DISPLAY_MODE_EMAIL_SETUP:
                    // Clear email buffer
                    memset(emailBuffer, 0, sizeof(emailBuffer));
                    emailIdx = 0;
                    displayNeedsUpdate = true;
                    break;

                case DISPLAY_MODE_CALCULATOR:
                    // Clear calculator
                    CalculatorClear();
                    displayNeedsUpdate = true;
                    break;

                case DISPLAY_MODE_SYSTEM_INFO:
                    // Refresh system info
                    xSemaphoreGive(dataMutexHandle);
                    const char* refreshMsg = "TEST:REQUEST_IP\n";
                    HAL_UART_Transmit(&huart6, (uint8_t*)refreshMsg, strlen(refreshMsg), 100);
                    HAL_GPIO_TogglePin(LED_PORT, LED_GREEN_PIN);
                    return;

                case DISPLAY_MODE_EMAIL_STATUS:
                case DISPLAY_MODE_SETTINGS:
                case DISPLAY_MODE_GAME:
                    // No secondary action needed
                    break;
                    
                default:
                    break;
            }
        }
        xSemaphoreGive(dataMutexHandle);
        RequestDisplayUpdate();
    }
}

void HandleDownButton(void) {
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (displayMode == DISPLAY_MODE_GAME) {
      // In game mode - Down button for game control
      Game_HandleInput(4); // 4 = Down button
        } else {
            // Left button - Navigate left or backspace
            switch(displayMode) {
                case DISPLAY_MODE_HOME:
                    // Navigate left (previous item)
                    carouselPosition = (carouselPosition - 1 + DESKTOP_ITEM_COUNT) % DESKTOP_ITEM_COUNT;
                    selectedDesktopItem = carouselPosition;
                    displayNeedsUpdate = true;
                    break;

                case DISPLAY_MODE_TEXT:
                    // Backspace - remove last character from text buffer
                    if (textBuffer.length > 0) {
                        textBuffer.length--;
                        textBuffer.text[textBuffer.length] = '\0';
                        displayNeedsUpdate = true;
                    }
                    break;

                case DISPLAY_MODE_EMAIL_SETUP:
                    // Backspace - remove last character from email
                    if (emailIdx > 0) {
                        emailIdx--;
                        emailBuffer[emailIdx] = '\0';
                        displayNeedsUpdate = true;
                    }
                    break;

                case DISPLAY_MODE_CALCULATOR:
                    // Backspace in calculator
                    CalculatorBackspace();
                    displayNeedsUpdate = true;
                    break;

                case DISPLAY_MODE_SYSTEM_INFO:
                case DISPLAY_MODE_SETTINGS:
                case DISPLAY_MODE_EMAIL_STATUS:
                case DISPLAY_MODE_GAME:
                    // No action - use PC1 to exit
                    break;
                    
                default:
                    break;
            }
        }
        xSemaphoreGive(dataMutexHandle);
        RequestDisplayUpdate();
    }
}

void DisplaySettingsScreen(void) {
    SH1106_GotoXY(2, 1);
    SH1106_Puts("<", &Font_7x10, 0);

    SH1106_GotoXY(18, 1);
    SH1106_Puts("WiFi Settings", &Font_7x10, 0);

    // Copy settings locally
    char localSSID[MAX_SSID_LENGTH + 1];
    char localPassword[MAX_PASSWORD_LENGTH + 1];
    char localStatus[32];
    bool editingSSID;
    uint8_t ssidLen, passLen;

    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        strncpy(localSSID, wifiSettings.ssid, MAX_SSID_LENGTH);
        localSSID[MAX_SSID_LENGTH] = '\0';
        strncpy(localPassword, wifiSettings.password, MAX_PASSWORD_LENGTH);
        localPassword[MAX_PASSWORD_LENGTH] = '\0';
        strncpy(localStatus, wifiSettings.status, 31);
        localStatus[31] = '\0';
        editingSSID = wifiSettings.editingSSID;
        ssidLen = wifiSettings.ssidIdx;
        passLen = wifiSettings.passwordIdx;
        xSemaphoreGive(dataMutexHandle);
    } else {
        strcpy(localSSID, "");
        strcpy(localPassword, "");
        strcpy(localStatus, "");
        editingSSID = true;
        ssidLen = 0;
        passLen = 0;
    }

    // SSID Input Field
    SH1106_DrawRectangle(2, 14, 124, 20, editingSSID ? 1 : 0);
    SH1106_GotoXY(5, 16);
    SH1106_Puts("SSID:", &Font_7x10, 1);

    if (ssidLen > 0) {
        SH1106_GotoXY(5, 25);
        char displaySSID[18];
        if (ssidLen > 17) {
            strncpy(displaySSID, localSSID + (ssidLen - 17), 17);
            displaySSID[17] = '\0';
        } else {
            strncpy(displaySSID, localSSID, ssidLen);
            displaySSID[ssidLen] = '\0';
        }
        SH1106_Puts(displaySSID, &Font_7x10, 1);

        // Cursor for SSID
        if (editingSSID && (HAL_GetTick() / 500) % 2) {
            uint8_t cursorX = 5 + (ssidLen > 17 ? 17 : ssidLen) * 7;
            SH1106_DrawLine(cursorX, 33, cursorX + 5, 33, 1);
        }
    } else if (editingSSID) {
        SH1106_GotoXY(5, 25);
        SH1106_Puts("Enter WiFi name", &Font_7x10, 1);
    }

    // Password Input Field
    SH1106_DrawRectangle(2, 36, 124, 20, !editingSSID ? 1 : 0);
    SH1106_GotoXY(5, 38);
    SH1106_Puts("Password:", &Font_7x10, 1);

    if (passLen > 0) {
        SH1106_GotoXY(5, 47);
        // Show asterisks for password
        char displayPass[18];
        uint8_t displayLen = (passLen > 17) ? 17 : passLen;
        for (uint8_t i = 0; i < displayLen; i++) {
            displayPass[i] = '*';
        }
        displayPass[displayLen] = '\0';
        SH1106_Puts(displayPass, &Font_7x10, 1);

        // Cursor for Password
        if (!editingSSID && (HAL_GetTick() / 500) % 2) {
            uint8_t cursorX = 5 + displayLen * 7;
            SH1106_DrawLine(cursorX, 55, cursorX + 5, 55, 1);
        }
    } else if (!editingSSID) {
        SH1106_GotoXY(5, 47);
        SH1106_Puts("Enter password", &Font_7x10, 1);
    }

    // Status message
    if (strlen(localStatus) > 0) {
        SH1106_GotoXY(5, 58);
        SH1106_Puts(localStatus, &Font_7x10, 1);
    }
}

// NEW: Handle Settings Input
void HandleSettingsInput(char c) {
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (wifiSettings.editingSSID) {
            // Editing SSID
            if (wifiSettings.ssidIdx < MAX_SSID_LENGTH) {
                wifiSettings.ssid[wifiSettings.ssidIdx++] = c;
                wifiSettings.ssid[wifiSettings.ssidIdx] = '\0';
                textBuffer.needsUpdate = true;
            }
        } else {
            // Editing Password
            if (wifiSettings.passwordIdx < MAX_PASSWORD_LENGTH) {
                wifiSettings.password[wifiSettings.passwordIdx++] = c;
                wifiSettings.password[wifiSettings.passwordIdx] = '\0';
                textBuffer.needsUpdate = true;
            }
        }
        xSemaphoreGive(dataMutexHandle);
    }
}

// NEW: Send WiFi Configuration to ESP
void SendWiFiConfigToESP(void) {
    char wifiCmd[128];
    int cmdLen;

    // Send SSID
    cmdLen = snprintf(wifiCmd, sizeof(wifiCmd), "WIFI_SSID:%s\n", wifiSettings.ssid);
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_UART_Transmit(&huart6, (uint8_t*)wifiCmd, cmdLen, 1000);
        xSemaphoreGive(dataMutexHandle);
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Send Password
    cmdLen = snprintf(wifiCmd, sizeof(wifiCmd), "WIFI_PASS:%s\n", wifiSettings.password);
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_UART_Transmit(&huart6, (uint8_t*)wifiCmd, cmdLen, 1000);
        xSemaphoreGive(dataMutexHandle);
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Send connect command
    const char* connectCmd = "WIFI_CONNECT\n";
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_UART_Transmit(&huart6, (uint8_t*)connectCmd, strlen(connectCmd), 1000);
        xSemaphoreGive(dataMutexHandle);
    }

    HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
}

void DisplayTask(void *argument) {
  DisplayMode_t lastMode = DISPLAY_MODE_HOME;
  bool lastGameMode = false;

  // Small delay to let system stabilize
  vTaskDelay(pdMS_TO_TICKS(500));

  for (;;) {
    bool updateNeeded = false;
    DisplayMode_t currentMode;
    bool currentGameMode;

    // Check if update is needed
    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
      updateNeeded = displayNeedsUpdate || textBuffer.needsUpdate;
      displayNeedsUpdate = false;
      textBuffer.needsUpdate = false;
      currentMode = displayMode;
      currentGameMode = gameMode;  // Get current game mode state
      xSemaphoreGive(dataMutexHandle);
    } else {
      // Couldn't get mutex, try again next cycle
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Update if:
    // 1. Display mode changed
    // 2. Game mode state changed (entered or exited game)
    // 3. Normal display update is needed
    // 4. In game mode, always update for smooth animation
    // 5. In DISPLAY_MODE_GAME (menu or playing), always update
    if (currentMode != lastMode || currentGameMode != lastGameMode || updateNeeded || currentGameMode || currentMode == DISPLAY_MODE_GAME) {
      updateNeeded = true;
      lastMode = currentMode;
      lastGameMode = currentGameMode;
    }

    if (updateNeeded) {
      if (currentGameMode || currentMode == DISPLAY_MODE_GAME) {
        // Game mode (playing) or game display mode (menu) - use separate rendering path
        if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
          // Game_Render() handles screen clearing internally
          Game_Render();
          SH1106_UpdateScreen();
          xSemaphoreGive(dataMutexHandle);
        }
      } else {
        // Normal display mode
        UpdateDisplay();
      }
      // Small delay after update to let I2C settle
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void UpdateDisplay(void) {
  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }

  DisplayMode_t currentMode = displayMode;
  displayNeedsUpdate = false;
  xSemaphoreGive(dataMutexHandle);

  // Now update display WITHOUT holding data mutex
  SH1106_Clear();

  switch(currentMode) {
    case DISPLAY_MODE_HOME:
      DisplayHomeScreen();
      break;
    case DISPLAY_MODE_TEXT:
      DisplayTextScreen();
      break;
    case DISPLAY_MODE_EMAIL_SETUP:
      DisplayEmailSetupScreen();
      break;
    case DISPLAY_MODE_EMAIL_STATUS:
      DisplayEmailStatusScreen();
      break;
    case DISPLAY_MODE_SYSTEM_INFO:
      DisplaySystemInfoScreen();
      break;
    case DISPLAY_MODE_CALCULATOR:
      DisplayCalculatorScreen();
      break;
    case DISPLAY_MODE_GAME:
      // Game rendering is handled by DisplayUpdateTask when gameMode is true
      // This case is only reached if gameMode is false (shouldn't happen normally)
      if (!gameMode) {
        Game_Menu_Render();
      }
      break;
    case DISPLAY_MODE_SETTINGS:
      DisplaySettingsScreen();
      break;
  }

  SH1106_UpdateScreen();
}

void UARTReceiveTask(void *argument) {
  char buffer[128];
  uint8_t idx = 0;
  uint8_t c;

  memset(buffer, 0, sizeof(buffer));

  for(;;) {
    if (HAL_UART_Receive(&huart6, &c, 1, 10) == HAL_OK) {
      if (c == '\r') continue;

      if (c == '\n') {
        if (idx > 0) {
          buffer[idx] = '\0';

          if (strncmp(buffer, "IP:", 3) == 0) {
            char* ipStr = buffer + 3;
            int ipLen = strlen(ipStr);

            if (ipLen > 0 && ipLen < 19) {
              if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
                strncpy(ipEsp, ipStr, 19);
                ipEsp[19] = '\0';

                if (displayMode == DISPLAY_MODE_SYSTEM_INFO) {
                  displayNeedsUpdate = true;
                }

                xSemaphoreGive(dataMutexHandle);
                RequestDisplayUpdate();
              }

              HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
              vTaskDelay(pdMS_TO_TICKS(100));
              HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
            }
          }

          else if (strncmp(buffer, "EMAIL:", 6) == 0) {
            char* statusStr = buffer + 6;
            int statusLen = strlen(statusStr);

            if (statusLen > 0 && statusLen < 31) {
              if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
                strncpy(emailInfo.status, statusStr, 31);
                emailInfo.status[31] = '\0';

                if (displayMode != DISPLAY_MODE_EMAIL_STATUS) {
                  displayMode = DISPLAY_MODE_EMAIL_STATUS;
                }
                displayNeedsUpdate = true;

                xSemaphoreGive(dataMutexHandle);

                if (strstr(statusStr, "success") != NULL ||
                    strstr(statusStr, "Sent") != NULL ||
                    strstr(statusStr, "sent") != NULL) {
                  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
                }
                else if (strstr(statusStr, "fail") != NULL ||
                         strstr(statusStr, "Error") != NULL ||
                         strstr(statusStr, "failed") != NULL) {
                  HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
                }
                else if (strstr(statusStr, "Sending") != NULL ||
                         strstr(statusStr, "Preparing") != NULL ||
                         strstr(statusStr, "Connecting") != NULL ||
                         strstr(statusStr, "Authenticating") != NULL) {
                  HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
                }

                RequestDisplayUpdate();
              }
            }
          }

          else if (strncmp(buffer, "ACK:", 4) == 0) {
            HAL_GPIO_TogglePin(LED_PORT, LED_BLUE_PIN);
          }

          idx = 0;
          memset(buffer, 0, sizeof(buffer));
        }
      }
      else if (idx < sizeof(buffer) - 1) {
        buffer[idx++] = c;
      }
      else {
        idx = 0;
        memset(buffer, 0, sizeof(buffer));
        HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(50));
        HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void DisplayHomeScreen(void) {
    SH1106_DrawBitmap(0, 0, bg, 128, 64, 1);

    SH1106_GotoXY(18, 1);
    SH1106_Puts("Home", &Font_7x10, 0);

    // Carousel settings
    const uint8_t iconWidth = 16;
    const uint8_t spacing = 35;
    const uint8_t startX = 57;
    const uint8_t startY = 20;
    const uint8_t selectedScale = 1.2; // Enlarge selected icon 2×

    // Draw three items centered around carouselPosition
    for (int8_t i = -1; i <= 1; i++) {
        int8_t itemIndex = (carouselPosition + i + DESKTOP_ITEM_COUNT) % DESKTOP_ITEM_COUNT;
        uint8_t x = startX + (i * spacing);
        uint8_t y = startY;

        if (i == 0) { // Selected item (center)
            x -= (iconWidth * (selectedScale - 1)) / 2;
            y -= 4;

            SH1106_DrawBitmap(x, y, desktopItems[itemIndex].icon, 16, 16, 1);


            // Label under icon
            uint8_t labelLen = strlen(desktopItems[itemIndex].name);
            uint8_t labelX = x + (iconWidth * selectedScale - labelLen * 7) / 2;
            SH1106_GotoXY(labelX, y + iconWidth * selectedScale + 4);
            SH1106_Puts((char*)desktopItems[itemIndex].name, &Font_7x10, 1);

        } else {
            // Non-selected icons
        	SH1106_DrawBitmap(x, y, desktopItems[itemIndex].icon, 16, 16, 1);
        }
    }
}

void DisplaySystemInfoScreen(void) {
    SH1106_GotoXY(2, 1);
    SH1106_Puts("<", &Font_7x10, 0);
    SH1106_GotoXY(18, 1);
    SH1106_Puts("System Info", &Font_7x10, 0);

    uint8_t yPos = 14;

    // Row 1: Keyboard and WiFi status
    // Keyboard
    SH1106_GotoXY(2, yPos + 3);
    SH1106_Puts("KB:", &Font_7x10, 1);
    if (keyboardConnected) {
        // Draw checkmark
        SH1106_DrawLine(38, yPos + 7, 40, yPos + 9, 1);
        SH1106_DrawLine(40, yPos + 9, 44, yPos + 4, 1);
    } else {
        // Draw X
        SH1106_DrawLine(38, yPos + 4, 43, yPos + 9, 1);
        SH1106_DrawLine(43, yPos + 4, 38, yPos + 9, 1);
    }

    // WiFi
    SH1106_GotoXY(50, yPos + 3);
    SH1106_Puts("WiFi:", &Font_7x10, 1);
    if (strlen(ipEsp) > 0) {
        // Draw signal bars
        SH1106_DrawLine(100, yPos + 9, 100, yPos + 9, 1);
        SH1106_DrawLine(103, yPos + 7, 103, yPos + 9, 1);
        SH1106_DrawLine(106, yPos + 5, 106, yPos + 9, 1);
    } else {
        // Draw X
        SH1106_DrawLine(100, yPos + 4, 105, yPos + 9, 1);
        SH1106_DrawLine(105, yPos + 4, 100, yPos + 9, 1);
    }

    yPos += 17;

    // Row 2: IP Address
    SH1106_GotoXY(2, yPos);
    SH1106_Puts("IP:", &Font_7x10, 1);
    SH1106_GotoXY(20, yPos);
    if (strlen(ipEsp) > 0) {
        SH1106_Puts(ipEsp, &Font_7x10, 1);
    } else {
        SH1106_Puts("Not Connected", &Font_7x10, 1);
    }

    yPos += 11;

    // Row 3: Temperature and Uptime
    SH1106_GotoXY(2, yPos);
    SH1106_Puts("Temp:", &Font_7x10, 1);
    SH1106_GotoXY(38, yPos);
    SH1106_Puts("42.1", &Font_7x10, 1);

    // Uptime
    SH1106_GotoXY(72, yPos);
    SH1106_Puts("Up:", &Font_7x10, 1);
    char uptimeStr[8];
    uint32_t uptimeMinutes = HAL_GetTick() / 60000;
    if (uptimeMinutes < 60) {
        snprintf(uptimeStr, sizeof(uptimeStr), "%lum", uptimeMinutes);
    } else if (uptimeMinutes < 1440) {
        snprintf(uptimeStr, sizeof(uptimeStr), "%luh", uptimeMinutes / 60);
    } else {
        snprintf(uptimeStr, sizeof(uptimeStr), "%lud", uptimeMinutes / 1440);
    }
    SH1106_GotoXY(93, yPos);
    SH1106_Puts(uptimeStr, &Font_7x10, 1);

    yPos += 11;

    // Row 4: RAM
    SH1106_GotoXY(2, yPos);
    SH1106_Puts("RAM:", &Font_7x10, 1);
    char ramStr[12];
    extern uint32_t _end;
    extern uint32_t _estack;
    uint32_t freeRam = (uint32_t)&_estack - (uint32_t)&_end;
    snprintf(ramStr, sizeof(ramStr), "%luKB", freeRam / 1024);
    SH1106_GotoXY(32, yPos);
    SH1106_Puts(ramStr, &Font_7x10, 1);

    // Flash
    SH1106_GotoXY(72, yPos);
    SH1106_Puts("MF:", &Font_7x10, 1);
    extern uint32_t _sidata;
    uint32_t flashUsed = (uint32_t)&_sidata;
    snprintf(ramStr, sizeof(ramStr), "%luKB", flashUsed / 1024/ 1024);
    SH1106_GotoXY(90, yPos);
    SH1106_Puts(ramStr, &Font_7x10, 1);
}
void DisplayTextScreen(void) {
    SH1106_GotoXY(2, 1);
    SH1106_Puts("<", &Font_7x10, 0);

    SH1106_DrawBitmap(16, 0, icon_text, 10, 10,1);

    SH1106_GotoXY(28, 1);
    SH1106_Puts("Text Input", &Font_7x10, 0);

    // Character counter
    char countStr[8];
    snprintf(countStr, sizeof(countStr), "%d", textBuffer.length);
    SH1106_GotoXY(105, 1);
    SH1106_Puts(countStr, &Font_7x10, 0);

    // Copy text buffer locally
    char localText[MAX_TEXT_LENGTH];
    uint16_t localLength;

    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        localLength = textBuffer.length;
        if (localLength > 0) {
            memcpy(localText, textBuffer.text, localLength);
            localText[localLength] = '\0';
        }
        xSemaphoreGive(dataMutexHandle);
    } else {
        localLength = 0;
    }

    if (localLength > 0) {
        int maxDisplayChars = DISPLAY_CHARS_PER_LINE * 4;
        int startPos = 0;

        if (localLength > maxDisplayChars) {
            startPos = localLength - maxDisplayChars;
        }

        char displayLine[DISPLAY_CHARS_PER_LINE + 1];
        int yPos = 14;
        int lineNum = 0;
        int charCount = 0;

        for (int i = startPos; i < localLength && lineNum < 4; i++) {
            char c = localText[i];

            if (c == '\n') {
                if (charCount > 0) {
                    displayLine[charCount] = '\0';
                    SH1106_GotoXY(2, yPos);
                    SH1106_Puts(displayLine, &Font_7x10, 1);
                    yPos += 10;
                    lineNum++;
                }
                charCount = 0;
            }
            else if (charCount >= DISPLAY_CHARS_PER_LINE) {
                displayLine[charCount] = '\0';
                SH1106_GotoXY(2, yPos);
                SH1106_Puts(displayLine, &Font_7x10, 1);
                yPos += 10;
                lineNum++;
                charCount = 0;
                displayLine[charCount++] = c;
            }
            else {
                displayLine[charCount++] = c;
            }
        }

        if (charCount > 0 && lineNum < 4) {
            displayLine[charCount] = '\0';
            SH1106_GotoXY(2, yPos);
            SH1106_Puts(displayLine, &Font_7x10, 1);
        }

        // Cursor indicator
        SH1106_DrawRectangle(120, 14, 6, 38, 1);
        uint8_t cursorY = 15 + ((localLength % 100) * 36 / 100);
        SH1106_DrawRectangle(121, cursorY, 4, 4, 1);

    } else {
        SH1106_GotoXY(12, 25);
        SH1106_Puts("Type your text", &Font_7x10, 1);
        SH1106_GotoXY(8, 35);
        SH1106_Puts("using keyboard...", &Font_7x10, 1);
    }

}

void DisplayCalculatorScreen(void) {
    // Copy display locally
    char localDisplay[22];
    char localOperator = 0;
    bool localError = false;

    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        strncpy(localDisplay, calcDisplay, 21);
        localDisplay[21] = '\0';
        localOperator = lastOperator;
        localError = errorState;
        xSemaphoreGive(dataMutexHandle);
    } else {
        strcpy(localDisplay, "0");
    }

    // LEFT SECTION (0-63): Operations/Buttons
    // Draw border around left section
    SH1106_DrawRectangle(0, 0, 64, 64, 1);

    // Title at top
    SH1106_GotoXY(4, 2);
    SH1106_Puts("Keys", &Font_7x10, 1);

    // Draw calculator buttons grid
    uint8_t buttonSize = 10;
    uint8_t buttonSpacing = 14;
    uint8_t gridStartX = 6;
    uint8_t buttonStartY = 14;

    // Button grid layout (4x4)
    // Row 1: 7 8 9 /
    SH1106_DrawRectangle(gridStartX, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + 2, buttonStartY + 1);
    SH1106_Puts("7", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing + 2, buttonStartY + 1);
    SH1106_Puts("8", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing * 2, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing * 2 + 2, buttonStartY + 1);
    SH1106_Puts("9", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing * 3, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing * 3 + 2, buttonStartY + 1);
    SH1106_Puts("/", &Font_7x10, 1);

    // Row 2: 4 5 6 *
    buttonStartY += buttonSpacing;
    SH1106_DrawRectangle(gridStartX, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + 2, buttonStartY + 1);
    SH1106_Puts("4", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing + 2, buttonStartY + 1);
    SH1106_Puts("5", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing * 2, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing * 2 + 2, buttonStartY + 1);
    SH1106_Puts("6", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing * 3, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing * 3 + 2, buttonStartY + 1);
    SH1106_Puts("*", &Font_7x10, 1);

    // Row 3: 1 2 3 -
    buttonStartY += buttonSpacing;
    SH1106_DrawRectangle(gridStartX, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + 2, buttonStartY + 1);
    SH1106_Puts("1", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing + 2, buttonStartY + 1);
    SH1106_Puts("2", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing * 2, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing * 2 + 2, buttonStartY + 1);
    SH1106_Puts("3", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing * 3, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing * 3 + 2, buttonStartY + 1);
    SH1106_Puts("-", &Font_7x10, 1);

    // Row 4: C 0 = +
    buttonStartY += buttonSpacing;
    SH1106_DrawRectangle(gridStartX, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + 2, buttonStartY + 1);
    SH1106_Puts("C", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing + 2, buttonStartY + 1);
    SH1106_Puts("0", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing * 2, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing * 2 + 2, buttonStartY + 1);
    SH1106_Puts("=", &Font_7x10, 1);

    SH1106_DrawRectangle(gridStartX + buttonSpacing * 3, buttonStartY, buttonSize, buttonSize, 1);
    SH1106_GotoXY(gridStartX + buttonSpacing * 3 + 2, buttonStartY + 1);
    SH1106_Puts("+", &Font_7x10, 1);

    // RIGHT SECTION (64-127): Result Display
    // Draw border around right section
    SH1106_DrawRectangle(64, 0, 64, 64, 1);

    // Calculator icon and title
    SH1106_DrawBitmap(68, 2, icon_calculator, 10, 10, 1);
    SH1106_GotoXY(80, 2);
    SH1106_Puts("Calc", &Font_7x10, 1);

    // Large display area for result
    SH1106_DrawRectangle(68, 16, 56, 44, 1);
    SH1106_Fill_Rectangle(69, 17, 54, 42, 0); // Clear inside

    // Operator indicator (small, top of display)
    if (localOperator != 0 && !localError) {
        char opStr[2] = {localOperator, '\0'};
        SH1106_GotoXY(70, 19);
        SH1106_Puts(opStr, &Font_7x10, 1);
    }

    // Display number - centered vertically in display area
    uint8_t dispLen = strlen(localDisplay);

    // For multi-line display if needed
    if (dispLen <= 8) {
        // Single line, centered
        uint8_t xPos = 120 - (dispLen * 7);
        if (xPos < 72) xPos = 72;
        SH1106_GotoXY(xPos, 35);
        SH1106_Puts(localDisplay, &Font_7x10, 1);
    } else {
        // Split into two lines if too long
        char line1[9];
        char line2[13];
        strncpy(line1, localDisplay, 8);
        line1[8] = '\0';
        strcpy(line2, localDisplay + 8);

        uint8_t xPos1 = 120 - (8 * 7);
        SH1106_GotoXY(xPos1, 28);
        SH1106_Puts(line1, &Font_7x10, 1);

        uint8_t len2 = strlen(line2);
        uint8_t xPos2 = 120 - (len2 * 7);
        if (xPos2 < 72) xPos2 = 72;
        SH1106_GotoXY(xPos2, 42);
        SH1106_Puts(line2, &Font_7x10, 1);
    }
}
void DisplayEmailSetupScreen(void) {

    SH1106_GotoXY(2, 1);
    SH1106_Puts("<", &Font_7x10, 0);

    SH1106_DrawBitmap(16, 0, icon_email, 10, 10,1);

    SH1106_GotoXY(28, 1);
    SH1106_Puts("Email Setup", &Font_7x10, 0);

    // Input field box
    SH1106_DrawRectangle(2, 15, 124, 22, 1);
    SH1106_GotoXY(5, 17);
    SH1106_Puts("To:", &Font_7x10, 1);

    // Copy email buffer locally
    char localEmailBuffer[64];
    uint8_t localEmailIdx;

    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        localEmailIdx = emailIdx;
        if (localEmailIdx > 0) {
            memcpy(localEmailBuffer, emailBuffer, localEmailIdx);
            localEmailBuffer[localEmailIdx] = '\0';
        }
        xSemaphoreGive(dataMutexHandle);
    } else {
        localEmailIdx = 0;
    }

    if (localEmailIdx > 0) {
        SH1106_GotoXY(5, 27);
        char displayEmail[20];
        if (localEmailIdx > 19) {
            strncpy(displayEmail, localEmailBuffer + (localEmailIdx - 19), 19);
            displayEmail[19] = '\0';
        } else {
            strncpy(displayEmail, localEmailBuffer, localEmailIdx);
            displayEmail[localEmailIdx] = '\0';
        }
        SH1106_Puts(displayEmail, &Font_7x10, 1);

        // Blinking cursor
        if ((HAL_GetTick() / 500) % 2) {
            uint8_t cursorX = 5 + (localEmailIdx > 19 ? 19 : localEmailIdx) * 7;
            SH1106_DrawLine(cursorX, 35, cursorX + 5, 35, 1);
        }
    } else {
        SH1106_GotoXY(5, 27);
        SH1106_Puts("email@example.com", &Font_7x10, 1);
    }

    // Instructions
    SH1106_GotoXY(10, 42);
    SH1106_Puts("Type recipient", &Font_7x10, 1);

}

void DisplayEmailStatusScreen(void) {
    SH1106_DrawBitmap(0, 0, bg, 128, 64, 1);


    SH1106_DrawBitmap(40, 0, icon_status, 10, 10,1);

    SH1106_GotoXY(52, 1);
    SH1106_Puts("Status", &Font_7x10, 0);

    // Copy status locally
    char localStatus[32];
    char localRecipient[64];

    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        strncpy(localStatus, emailInfo.status, 31);
        localStatus[31] = '\0';
        strncpy(localRecipient, emailInfo.recipient, 63);
        localRecipient[63] = '\0';
        xSemaphoreGive(dataMutexHandle);
    } else {
        strcpy(localStatus, "Error");
        strcpy(localRecipient, "Unknown");
    }

    // Status icon and message
    uint8_t iconY = 20;

    if (strstr(localStatus, "success") != NULL || strstr(localStatus, "Sent") != NULL) {
        // Success checkmark
        SH1106_DrawCircle(64, iconY + 6, 10, 1);
        SH1106_DrawLine(58, iconY + 6, 62, iconY + 10, 1);
        SH1106_DrawLine(62, iconY + 10, 70, iconY + 2, 1);

        SH1106_GotoXY(32, iconY + 20);
        SH1106_Puts("SUCCESS!", &Font_7x10, 1);
    }
    else if (strstr(localStatus, "fail") != NULL || strstr(localStatus, "Error") != NULL) {
        // Error X
        SH1106_DrawCircle(64, iconY + 6, 10, 1);
        SH1106_DrawLine(58, iconY, 70, iconY + 12, 1);
        SH1106_DrawLine(70, iconY, 58, iconY + 12, 1);

        SH1106_GotoXY(40, iconY + 20);
        SH1106_Puts("FAILED!", &Font_7x10, 1);
    }
    else {
        // Sending spinner (animated)
        uint8_t frame = (HAL_GetTick() / 200) % 8;
        // Simple rotating line animation
        SH1106_DrawCircle(64, iconY + 6, 8, 1);

        SH1106_GotoXY(28, iconY + 20);
        SH1106_Puts("Sending...", &Font_7x10, 1);
    }

    // Recipient info box
    SH1106_DrawRectangle(2, 42, 124, 10, 1);
    SH1106_GotoXY(5, 44);
    SH1106_Puts("To: ", &Font_7x10, 1);

    char displayEmail[17];
    strncpy(displayEmail, localRecipient, 16);
    displayEmail[16] = '\0';
    SH1106_GotoXY(28, 44);
    SH1106_Puts(displayEmail, &Font_7x10, 1);

}

void ESP_SendTask(void *argument) {
  for(;;) {
    if (xSemaphoreTake(dataMutexHandle, portMAX_DELAY) == pdTRUE) {
      xSemaphoreGive(dataMutexHandle);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void ESP_ReceiveTask(void *argument) {
  char buffer[128];
  uint8_t idx = 0;
  uint8_t c;
  uint32_t lastRxTime = 0;

  // Clear buffer on startup
  memset(buffer, 0, sizeof(buffer));

  for(;;) {
    // Check for RX overflow
    if (rxOverflow) {
      rxOverflow = false;

      // Signal error with RED LED
      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      vTaskDelay(pdMS_TO_TICKS(100));
      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);

      // Clear line buffer
      idx = 0;
      memset(buffer, 0, sizeof(buffer));
    }

    // Read all available bytes from circular buffer
    while (UART_ReadByte(&c)) {
      lastRxTime = HAL_GetTick();

      if (c == '\r') continue;  // Skip carriage returns

      if (c == '\n') {
        if (idx > 0) {
          buffer[idx] = '\0';

          // ===== PARSE IP MESSAGE =====
          if (strncmp(buffer, "IP:", 3) == 0) {
            char* ipStr = buffer + 3;
            int ipLen = strlen(ipStr);

            if (ipLen > 0 && ipLen < 19) {
              if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
                strncpy(ipEsp, ipStr, 19);
                ipEsp[19] = '\0';

                // Flash all LEDs
                HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_ORANGE_PIN | LED_BLUE_PIN | LED_RED_PIN, GPIO_PIN_SET);
                displayMode = DISPLAY_MODE_SYSTEM_INFO;
                textBuffer.needsUpdate = true;

                xSemaphoreGive(dataMutexHandle);
              }

              vTaskDelay(pdMS_TO_TICKS(200));
              HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_ORANGE_PIN | LED_BLUE_PIN | LED_RED_PIN, GPIO_PIN_RESET);
            }
          }
          else if (strncmp(buffer, "WIFI:", 5) == 0) {
                      char* statusStr = buffer + 5;

                      if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
                        strncpy(wifiSettings.status, statusStr, 31);
                        wifiSettings.status[31] = '\0';

                        if (displayMode == DISPLAY_MODE_SETTINGS) {
                          textBuffer.needsUpdate = true;
                        }

                        xSemaphoreGive(dataMutexHandle);

                        // LED feedback
                        if (strstr(statusStr, "Connected") != NULL || strstr(statusStr, "Success") != NULL) {
                          HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
                          vTaskDelay(pdMS_TO_TICKS(500));
                          HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
                        } else if (strstr(statusStr, "Failed") != NULL || strstr(statusStr, "Error") != NULL) {
                          HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
                          vTaskDelay(pdMS_TO_TICKS(500));
                          HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
                        }
                      }
                    }
          // ===== PARSE EMAIL STATUS MESSAGE =====
          else if (strncmp(buffer, "EMAIL:", 6) == 0) {
            char* statusStr = buffer + 6;
            int statusLen = strlen(statusStr);

            if (statusLen > 0 && statusLen < 31) {
              if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
                strncpy(emailInfo.status, statusStr, 31);
                emailInfo.status[31] = '\0';

                if (displayMode != DISPLAY_MODE_EMAIL_STATUS) {
                  displayMode = DISPLAY_MODE_EMAIL_STATUS;
                }
                textBuffer.needsUpdate = true;

                xSemaphoreGive(dataMutexHandle);

                // Update LEDs based on status
                if (strstr(statusStr, "success") != NULL ||
                    strstr(statusStr, "Sent") != NULL ||
                    strstr(statusStr, "sent") != NULL) {
                  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN | LED_RED_PIN, GPIO_PIN_RESET);
                }
                else if (strstr(statusStr, "fail") != NULL ||
                         strstr(statusStr, "Error") != NULL ||
                         strstr(statusStr, "failed") != NULL) {
                  HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN | LED_GREEN_PIN, GPIO_PIN_RESET);
                }
                else if (strstr(statusStr, "Sending") != NULL ||
                         strstr(statusStr, "Preparing") != NULL ||
                         strstr(statusStr, "Connecting") != NULL ||
                         strstr(statusStr, "Authenticating") != NULL) {
                  HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_RED_PIN, GPIO_PIN_RESET);
                }
              }
            }
          }

          // ===== PARSE ACK MESSAGE =====
          else if (strncmp(buffer, "ACK:", 4) == 0) {
            HAL_GPIO_TogglePin(LED_PORT, LED_BLUE_PIN);
            vTaskDelay(pdMS_TO_TICKS(50));
            HAL_GPIO_TogglePin(LED_PORT, LED_BLUE_PIN);
          }

          // Clear buffer and index
          idx = 0;
          memset(buffer, 0, sizeof(buffer));
        }
      }
      else if (idx < sizeof(buffer) - 1) {
        buffer[idx++] = c;
      }
      else {
        // Line buffer overflow
        idx = 0;
        memset(buffer, 0, sizeof(buffer));
        HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(50));
        HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
      }
    }

    // Timeout check - clear partial line if no data for 2 seconds
    if (idx > 0 && (HAL_GetTick() - lastRxTime) > 2000) {
      idx = 0;
      memset(buffer, 0, sizeof(buffer));
    }

    // Task delay - now can be longer since interrupts handle reception
    vTaskDelay(pdMS_TO_TICKS(10));
  }}
void SendEmailCommand(void) {
  char emailCmd[128];
  int cmdLen = snprintf(emailCmd, sizeof(emailCmd), "SEND_EMAIL:%s\n", emailInfo.recipient);
  HAL_UART_Transmit(&huart6, (uint8_t*)emailCmd, cmdLen, 1000);
  HAL_GPIO_TogglePin(LED_PORT, LED_GREEN_PIN);
}

void KeyboardTask(void *argument) {
  uint8_t i, j;
  bool keyFound;
  for(;;) {
    if (USBH_HID_GetDeviceType(&hUsbHostFS) == HID_KEYBOARD) {
      if (!keyboardConnected) {
        keyboardConnected = true;
        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(200));
        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
      }
      keyboardInfo = USBH_HID_GetKeybdInfo(&hUsbHostFS);
      if (keyboardInfo != NULL) {
        bool currentShiftState = (keyboardInfo->lshift || keyboardInfo->rshift);
        for (i = 0; i < 6; i++) {
          if (keyboardInfo->keys[i] != 0) {
            keyFound = false;
            for (j = 0; j < 6; j++) {
              if (keyboardInfo->keys[i] == lastKeyState[j]) {
                keyFound = true;
                break;
              }
            }
            if (!keyFound) {
              uint8_t keyCode = keyboardInfo->keys[i];
              char c = 0;
              bool isEmailMode = false;
              bool isCalcMode = false;
              bool isSettingsMode = false;

              // Check modes safely
              if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE) {
                isEmailMode = emailMode;
                isCalcMode = (displayMode == DISPLAY_MODE_CALCULATOR);
                isSettingsMode = (displayMode == DISPLAY_MODE_SETTINGS);
                xSemaphoreGive(dataMutexHandle);
              }

              // Enter key handling
              if (keyCode == 0x28) {
                if (isCalcMode) {
                  CalculatorExecute();
                } else if (isEmailMode) {
                  c = '\n';
                  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (emailIdx < 63) {
                      emailBuffer[emailIdx++] = c;
                    }
                    textBuffer.needsUpdate = true;
                    xSemaphoreGive(dataMutexHandle);
                  }
                } else if (isSettingsMode) {
                  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    wifiSettings.editingSSID = !wifiSettings.editingSSID;
                    textBuffer.needsUpdate = true;
                    xSemaphoreGive(dataMutexHandle);
                  }
                } else {
                  c = '\n';
                  AddCharToBuffer(c);
                  SendCharToESP(c);
                }
              }
              // Backspace
              else if (keyCode == 0x2A) {
                if (isCalcMode) {
                  CalculatorBackspace();
                } else if (isEmailMode) {
                  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (emailIdx > 0) {
                      emailIdx--;
                      emailBuffer[emailIdx] = '\0';
                      textBuffer.needsUpdate = true;
                    }
                    xSemaphoreGive(dataMutexHandle);
                  }
                } else if (isSettingsMode) {
                  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (wifiSettings.editingSSID && wifiSettings.ssidIdx > 0) {
                      wifiSettings.ssidIdx--;
                      wifiSettings.ssid[wifiSettings.ssidIdx] = '\0';
                      textBuffer.needsUpdate = true;
                    } else if (!wifiSettings.editingSSID && wifiSettings.passwordIdx > 0) {
                      wifiSettings.passwordIdx--;
                      wifiSettings.password[wifiSettings.passwordIdx] = '\0';
                      textBuffer.needsUpdate = true;
                    }
                    xSemaphoreGive(dataMutexHandle);
                  }
                } else {
                  if (textBuffer.length > 0) {
                    if (xSemaphoreTake(dataMutexHandle, portMAX_DELAY) == pdTRUE) {
                      textBuffer.length--;
                      textBuffer.text[textBuffer.length] = '\0';
                      textBuffer.needsUpdate = true;
                      xSemaphoreGive(dataMutexHandle);
                    }
                  }
                }
                HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(50));
                HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
              }
              // Space
              else if (keyCode == 0x2C) {
                c = ' ';
                if (isEmailMode) {
                  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (emailIdx < 63) {
                      emailBuffer[emailIdx++] = c;
                      textBuffer.needsUpdate = true;
                    }
                    xSemaphoreGive(dataMutexHandle);
                  }
                } else if (isSettingsMode) {
                  HandleSettingsInput(c);
                } else {
                  AddCharToBuffer(c);
                  SendCharToESP(c);
                }
              }
              // All other keys
              else {
                c = ConvertHIDKeyToChar(keyCode, currentShiftState);
                if (c != 0) {
                  if (isCalcMode) {
                    HandleCalculatorInput(c);
                  } else if (isEmailMode) {
                    if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                      if (emailIdx < 63) {
                        emailBuffer[emailIdx++] = c;
                        textBuffer.needsUpdate = true;
                      }
                      xSemaphoreGive(dataMutexHandle);
                    }
                  } else if (isSettingsMode) {
                    HandleSettingsInput(c);
                  } else {
                    AddCharToBuffer(c);
                    SendCharToESP(c);
                  }
                  HAL_GPIO_WritePin(LED_PORT, LED_BLUE_PIN, GPIO_PIN_SET);
                  vTaskDelay(pdMS_TO_TICKS(50));
                  HAL_GPIO_WritePin(LED_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
                }
              }
            }
          }
        }
        memcpy(lastKeyState, keyboardInfo->keys, 6);
      }
    } else {
      if (keyboardConnected) {
        keyboardConnected = false;
        memset(lastKeyState, 0, 6);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void AddCharToBuffer(char c) {
  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
    if (textBuffer.length < MAX_TEXT_LENGTH - 1) {
      textBuffer.text[textBuffer.length++] = c;
      textBuffer.text[textBuffer.length] = '\0';
      textBuffer.needsUpdate = true;
    }
    xSemaphoreGive(dataMutexHandle);
  }
}


void SendCharToESP(char c) {
  char keyMsg[15];

  snprintf(keyMsg, sizeof(keyMsg), "KEY:%c\n", c);

  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
    HAL_UART_Transmit(&huart6, (uint8_t*)keyMsg, strlen(keyMsg), HAL_MAX_DELAY);
    xSemaphoreGive(dataMutexHandle);
  }
}

// Keep this function for external calls if needed
void ClearTextBuffer(void) {
  if (xSemaphoreTake(dataMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
    memset(textBuffer.text, 0, sizeof(textBuffer.text));
    textBuffer.length = 0;
    textBuffer.needsUpdate = true;
    xSemaphoreGive(dataMutexHandle);
  }
}

char ConvertHIDKeyToChar(uint8_t keyCode, bool shift) {
  if (keyCode >= 0x04 && keyCode <= 0x1D) {
    if (shift) {
      return 'A' + (keyCode - 0x04);
    } else {
      return 'a' + (keyCode - 0x04);
    }
  } else if (keyCode >= 0x1E && keyCode <= 0x27) {
    if (shift) {
      const char symbols[] = "!@#$%^&*()";
      return symbols[keyCode - 0x1E];
    } else {
      if (keyCode == 0x27) return '0';
      return '1' + (keyCode - 0x1E);
    }
  } else {
    switch (keyCode) {
      case 0x2D: return shift ? '_' : '-';
      case 0x2E: return shift ? '+' : '=';
      case 0x2F: return shift ? '{' : '[';
      case 0x30: return shift ? '}' : ']';
      case 0x31: return shift ? '|' : '\\';
      case 0x33: return shift ? ':' : ';';
      case 0x34: return shift ? '"' : '\'';
      case 0x35: return shift ? '~' : '`';
      case 0x36: return shift ? '<' : ',';
      case 0x37: return shift ? '>' : '.';
      case 0x38: return shift ? '?' : '/';
      case 0x64: return shift ? '_' : '@';
      default: return 0;
    }
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  MX_USB_HOST_Init();
  SH1106_Init();

  // Animated startup using boot animation frames
  // Play the animation (124x68 pixels, centered on 128x64 display)
  for(int i = 0; i < epd_bitmap_allArray_LEN; i++) {
    SH1106_Fill(SH1106_COLOR_BLACK);
    SH1106_DrawBitmap(2, -2, epd_bitmap_allArray[i], 124, 68, SH1106_COLOR_WHITE);
    SH1106_UpdateScreen();
    osDelay(50);  // 50ms delay per frame for smooth animation
  }

  osDelay(300);

  SH1106_Fill(SH1106_COLOR_BLACK);
  SH1106_GotoXY(38, 22);
  SH1106_Puts("HELLO!", &Font_7x10, SH1106_COLOR_WHITE);
  SH1106_DrawLine(10, 38, 118, 38, SH1106_COLOR_WHITE);
  SH1106_UpdateScreen();
  osDelay(400);

  SH1106_GotoXY(20, 48);
  SH1106_Puts("System Ready", &Font_7x10, SH1106_COLOR_WHITE);
  SH1106_UpdateScreen();
  osDelay(500);

  SH1106_Clear();
  DisplayHomeScreen();
  SH1106_UpdateScreen();

  for(;;) {
    USBH_Process(&hUsbHostFS);
    osDelay(200);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

