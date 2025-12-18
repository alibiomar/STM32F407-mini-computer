/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F407E-DISCO with OLED Display and HID Keyboard
  *                   PC1 = Switch/Navigate button
  *                   PC2 = Confirm/OK button (FIXED from PC0)
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_GREEN_PIN    GPIO_PIN_12
#define LED_ORANGE_PIN   GPIO_PIN_13
#define LED_RED_PIN      GPIO_PIN_14
#define LED_BLUE_PIN     GPIO_PIN_15
#define LED_PORT         GPIOD

#define BUTTON_CONFIRM_PIN   GPIO_PIN_2  // PC2 - Confirm/OK button (FIXED)
#define BUTTON_SWITCH_PIN    GPIO_PIN_1  // PC1 - Switch/Navigate button
#define BUTTON_PORT          GPIOC

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
QueueHandle_t espQueue;
SemaphoreHandle_t uartMutexHandle;
SemaphoreHandle_t displayMutexHandle;
SemaphoreHandle_t i2cMutexHandle;  // NEW: Separate mutex for I2C hardware access
SemaphoreHandle_t buttonSemaphore;

TaskHandle_t espSendTaskHandle;
TaskHandle_t espRecvTaskHandle;
TaskHandle_t keyboardTaskHandle;
TaskHandle_t displayTaskHandle;

// Button variables
GPIO_PinState lastSwitchState = GPIO_PIN_SET;
GPIO_PinState lastConfirmState = GPIO_PIN_SET;
uint32_t lastSwitchDebounceTime = 0;
uint32_t lastConfirmDebounceTime = 0;
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
char ipEsp[20] = {0};

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
void ESP_SendTask(void *argument);
void ESP_ReceiveTask(void *argument);
void KeyboardTask(void *argument);
void GameUpdateTask(void *argument);
void DisplayTask(void *argument);
void ButtonTask(void *argument);
bool ProcessLEDCommand(char* buffer, char* ledState);
void Initialize_Resources(void);
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
void HandleSwitchButton(void);
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
  // Adjusted priorities to prevent conflicts
  // Lower number = lower priority
  xTaskCreate(ESP_SendTask, "espSendTask", 256, NULL, 1, &espSendTaskHandle);
  xTaskCreate(KeyboardTask, "keyboardTask", 768, NULL, 2, &keyboardTaskHandle);
  xTaskCreate(ESP_ReceiveTask, "espRecvTask", 768, NULL, 3, &espRecvTaskHandle);
  xTaskCreate(DisplayTask, "displayTask", 768, NULL, 2, &displayTaskHandle);
  xTaskCreate(ButtonTask, "buttonTask", 256, NULL, 4, NULL);  // Highest priority
  xTaskCreate(GameUpdateTask, "gameUpdate", 256, NULL, 2, NULL);

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

  /*Configure GPIO pins : PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
  uartMutexHandle = xSemaphoreCreateMutex();
  if (uartMutexHandle == NULL) {
    Error_Handler();
  }

  displayMutexHandle = xSemaphoreCreateMutex();
  if (displayMutexHandle == NULL) {
    Error_Handler();
  }

  i2cMutexHandle = xSemaphoreCreateMutex();
  if (i2cMutexHandle == NULL) {
    Error_Handler();
  }
  Game_Init();
  // Start UART interrupt reception
  HAL_UART_Receive_IT(&huart6, &uartRxByte, 1);
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
    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        strcpy(calcDisplay, "0");
        memset(calcBuffer, 0, sizeof(calcBuffer));
        lastOperator = 0;
        lastValue = 0.0;
        newNumber = true;
        errorState = false;
        textBuffer.needsUpdate = true;
        xSemaphoreGive(displayMutexHandle);
    }
}

void CalculatorBackspace(void) {
    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t len = strlen(calcDisplay);
        if (len > 1 && !errorState) {
            calcDisplay[len - 1] = '\0';
        } else {
            strcpy(calcDisplay, "0");
        }
        textBuffer.needsUpdate = true;
        xSemaphoreGive(displayMutexHandle);
    }
}

void CalculatorExecute(void) {
    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (errorState) {
            xSemaphoreGive(displayMutexHandle);
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
                        xSemaphoreGive(displayMutexHandle);
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
        xSemaphoreGive(displayMutexHandle);
    }
}

void HandleCalculatorInput(char c) {
    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (errorState && c != 'C' && c != 'c') {
            xSemaphoreGive(displayMutexHandle);
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
            xSemaphoreGive(displayMutexHandle);
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
                            xSemaphoreGive(displayMutexHandle);
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

        xSemaphoreGive(displayMutexHandle);
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
  GPIO_PinState currentSwitchState;
  GPIO_PinState currentConfirmState;

  // Initial state reading
  lastSwitchState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_SWITCH_PIN);
  lastConfirmState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_CONFIRM_PIN);

  vTaskDelay(pdMS_TO_TICKS(100)); // Initial stabilization delay

  for(;;) {
    // Read button states
    currentSwitchState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_SWITCH_PIN);
    currentConfirmState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_CONFIRM_PIN);

    // Handle Switch Button (PC1) - Navigate through screens
    // Button pressed: HIGH to LOW transition (pull-up resistor)
    if (currentSwitchState == GPIO_PIN_RESET && lastSwitchState == GPIO_PIN_SET) {
      uint32_t currentTime = HAL_GetTick();
      if ((currentTime - lastSwitchDebounceTime) > DEBOUNCE_DELAY) {
        lastSwitchDebounceTime = currentTime;

        // Visual feedback - turn ON LED
        HAL_GPIO_WritePin(LED_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);

        // Handle button action
        HandleSwitchButton();

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

    lastSwitchState = currentSwitchState;
    lastConfirmState = currentConfirmState;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void HandleSwitchButton(void) {
    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (gameMode) {
            // In game mode - pass input to game system
            Game_HandleInput(1); // 1 = Switch button
        } else {
            // Normal mode handling
            switch(displayMode) {
                case DISPLAY_MODE_HOME:
                    // Cycle carousel right
                    carouselPosition = (carouselPosition + 1) % DESKTOP_ITEM_COUNT;
                    selectedDesktopItem = carouselPosition;
                    textBuffer.needsUpdate = true;
                    break;

                case DISPLAY_MODE_GAME:
                    // If in game menu but not in game mode, exit completely
                    displayMode = DISPLAY_MODE_HOME;
                    textBuffer.needsUpdate = true;
                    break;
                case DISPLAY_MODE_SETTINGS:  // UPDATED - Exit to home
                               displayMode = DISPLAY_MODE_HOME;
                               // Clear settings data
                               wifiSettings.editingSSID = true;
                               wifiSettings.ssidIdx = 0;
                               wifiSettings.passwordIdx = 0;
                               memset(wifiSettings.ssid, 0, sizeof(wifiSettings.ssid));
                               memset(wifiSettings.password, 0, sizeof(wifiSettings.password));
                               memset(wifiSettings.status, 0, sizeof(wifiSettings.status));
                               textBuffer.needsUpdate = true;
                               break;

                case DISPLAY_MODE_SYSTEM_INFO:
                case DISPLAY_MODE_TEXT:
                case DISPLAY_MODE_EMAIL_SETUP:
                case DISPLAY_MODE_EMAIL_STATUS:
                case DISPLAY_MODE_CALCULATOR:
                    // Return to home
                    displayMode = DISPLAY_MODE_HOME;
                    emailMode = false;
                    emailIdx = 0;
                    memset(emailBuffer, 0, sizeof(emailBuffer));
                    textBuffer.needsUpdate = true;
                    break;
                default:
                    displayMode = DISPLAY_MODE_HOME;
                    break;
            }
        }
        xSemaphoreGive(displayMutexHandle);
    }
}

void HandleConfirmButton(void) {
    DisplayMode_t currentMode;
    bool currentGameMode;
    bool needsClear = false;
    bool sendEmail = false;
    bool sendIPRequest = false;
    bool sendWiFiConfig = false;
    char emailRecipient[64] = {0};

    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        currentMode = displayMode;
        currentGameMode = gameMode;
        xSemaphoreGive(displayMutexHandle);
    } else {
        return;
    }

    if (currentGameMode) {
        Game_HandleInput(2); // 2 = Confirm button
        return;
    }

    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        switch(currentMode) {
            case DISPLAY_MODE_HOME:
                displayMode = desktopItems[selectedDesktopItem].targetMode;
                if (displayMode == DISPLAY_MODE_TEXT) {
                    emailMode = false;
                    emailInfo.textConfirmed = false;
                    emailInfo.hasRecipient = false;
                    needsClear = true;
                } else if (displayMode == DISPLAY_MODE_EMAIL_SETUP) {
                    emailMode = true;
                    emailIdx = 0;
                    memset(emailBuffer, 0, sizeof(emailBuffer));
                } else if (displayMode == DISPLAY_MODE_GAME) {
                    gameMode = true;
                    currentGame.state = GAME_STATE_MENU;
                } else if (displayMode == DISPLAY_MODE_SETTINGS) {
                    wifiSettings.editingSSID = true;
                    wifiSettings.ssidIdx = 0;
                    wifiSettings.passwordIdx = 0;
                    memset(wifiSettings.ssid, 0, sizeof(wifiSettings.ssid));
                    memset(wifiSettings.password, 0, sizeof(wifiSettings.password));
                    memset(wifiSettings.status, 0, sizeof(wifiSettings.status));
                }
                textBuffer.needsUpdate = true;
                break;

            case DISPLAY_MODE_GAME:
                gameMode = true;
                currentGame.state = GAME_STATE_MENU;
                textBuffer.needsUpdate = true;
                break;

            case DISPLAY_MODE_TEXT:
                if (textBuffer.length > 0) {
                    emailInfo.textConfirmed = true;
                    displayMode = DISPLAY_MODE_EMAIL_SETUP;
                    emailMode = true;
                    emailIdx = 0;
                    memset(emailBuffer, 0, sizeof(emailBuffer));
                    textBuffer.needsUpdate = true;
                }
                break;

            case DISPLAY_MODE_EMAIL_SETUP:
                if (emailIdx > 0) {
                    uint8_t copyLen = (emailIdx < 63) ? emailIdx : 63;
                    emailBuffer[copyLen] = '\0';
                    strncpy(emailInfo.recipient, emailBuffer, 63);
                    emailInfo.recipient[63] = '\0';
                    emailInfo.hasRecipient = true;
                    emailMode = false;
                    strncpy(emailRecipient, emailInfo.recipient, 63);
                    emailRecipient[63] = '\0';
                    sendEmail = true;
                    strcpy(emailInfo.status, "Sending...");
                    displayMode = DISPLAY_MODE_EMAIL_STATUS;
                    textBuffer.needsUpdate = true;
                }
                break;

            case DISPLAY_MODE_EMAIL_STATUS:
                HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN, GPIO_PIN_RESET);
                displayMode = DISPLAY_MODE_HOME;
                emailInfo.textConfirmed = false;
                emailInfo.hasRecipient = false;
                memset(emailInfo.status, 0, sizeof(emailInfo.status));
                memset(emailInfo.recipient, 0, sizeof(emailInfo.recipient));
                textBuffer.needsUpdate = true;
                needsClear = true;
                break;

            case DISPLAY_MODE_SETTINGS:
            	if (wifiSettings.ssidIdx > 0 ) {
            	                    strcpy(wifiSettings.status, "Updating...");
            	                    sendWiFiConfig = true;
            	                    textBuffer.needsUpdate = true;
            	                }
                break;

            case DISPLAY_MODE_SYSTEM_INFO:
                sendIPRequest = true;
                break;
        }

        if (needsClear) {
            memset(textBuffer.text, 0, sizeof(textBuffer.text));
            textBuffer.length = 0;
            textBuffer.needsUpdate = true;
        }
        xSemaphoreGive(displayMutexHandle);
    }

    if (sendIPRequest) {
        if (xSemaphoreTake(uartMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
            const char* refreshMsg = "TEST:REQUEST_IP\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)refreshMsg, strlen(refreshMsg), 1000);
            xSemaphoreGive(uartMutexHandle);
            HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(100));
            HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
        }
    }

    if (sendEmail) {
        vTaskDelay(pdMS_TO_TICKS(50));
        SendEmailCommand();
    }

    if (sendWiFiConfig) {
        vTaskDelay(pdMS_TO_TICKS(50));
        SendWiFiConfigToESP();
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

    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        strncpy(localSSID, wifiSettings.ssid, MAX_SSID_LENGTH);
        localSSID[MAX_SSID_LENGTH] = '\0';
        strncpy(localPassword, wifiSettings.password, MAX_PASSWORD_LENGTH);
        localPassword[MAX_PASSWORD_LENGTH] = '\0';
        strncpy(localStatus, wifiSettings.status, 31);
        localStatus[31] = '\0';
        editingSSID = wifiSettings.editingSSID;
        ssidLen = wifiSettings.ssidIdx;
        passLen = wifiSettings.passwordIdx;
        xSemaphoreGive(displayMutexHandle);
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
    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
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
        xSemaphoreGive(displayMutexHandle);
    }
}

// NEW: Send WiFi Configuration to ESP
void SendWiFiConfigToESP(void) {
    char wifiCmd[128];
    int cmdLen;

    // Send SSID
    cmdLen = snprintf(wifiCmd, sizeof(wifiCmd), "WIFI_SSID:%s\n", wifiSettings.ssid);
    if (xSemaphoreTake(uartMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_UART_Transmit(&huart6, (uint8_t*)wifiCmd, cmdLen, 1000);
        xSemaphoreGive(uartMutexHandle);
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Send Password
    cmdLen = snprintf(wifiCmd, sizeof(wifiCmd), "WIFI_PASS:%s\n", wifiSettings.password);
    if (xSemaphoreTake(uartMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_UART_Transmit(&huart6, (uint8_t*)wifiCmd, cmdLen, 1000);
        xSemaphoreGive(uartMutexHandle);
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Send connect command
    const char* connectCmd = "WIFI_CONNECT\n";
    if (xSemaphoreTake(uartMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_UART_Transmit(&huart6, (uint8_t*)connectCmd, strlen(connectCmd), 1000);
        xSemaphoreGive(uartMutexHandle);
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
    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
      updateNeeded = textBuffer.needsUpdate;
      textBuffer.needsUpdate = false;
      currentMode = displayMode;
      currentGameMode = gameMode;  // Get current game mode state
      xSemaphoreGive(displayMutexHandle);
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
    if (currentMode != lastMode || currentGameMode != lastGameMode || updateNeeded || currentGameMode) {
      updateNeeded = true;
      lastMode = currentMode;
      lastGameMode = currentGameMode;
    }

    if (updateNeeded) {
      if (currentGameMode) {
        // Game mode - use separate rendering path
        if (xSemaphoreTake(i2cMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
          Game_Render();
          SH1106_UpdateScreen();
          xSemaphoreGive(i2cMutexHandle);
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
  // Take mutex only to read display mode
  DisplayMode_t currentMode;
  bool currentGameMode;

  if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
    currentMode = displayMode;
    currentGameMode = gameMode;
    xSemaphoreGive(displayMutexHandle);
  } else {
    return;  // Failed to get mutex, skip this update
  }

  // If in game mode, let DisplayTask handle rendering
  if (currentGameMode) {
    return;
  }

  // Now do the actual display update with I2C mutex protection
  if (xSemaphoreTake(i2cMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
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
      case DISPLAY_MODE_GAME:  // Add this case
        Game_Menu_Render();
        break;
      case DISPLAY_MODE_SETTINGS:  // NEW
              DisplaySettingsScreen();
              break;
    }

    SH1106_UpdateScreen();
    xSemaphoreGive(i2cMutexHandle);
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

            DrawIconWithFrame(x, y, desktopItems[itemIndex].icon,1);


            // Label under icon
            uint8_t labelLen = strlen(desktopItems[itemIndex].name);
            uint8_t labelX = x + (iconWidth * selectedScale - labelLen * 7) / 2;
            SH1106_GotoXY(labelX, y + iconWidth * selectedScale + 4);
            SH1106_Puts(desktopItems[itemIndex].name, &Font_7x10, 1);

        } else {
            // Non-selected icons
            DrawIconWithFrame(x, y, desktopItems[itemIndex].icon,1);
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

    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        localLength = textBuffer.length;
        if (localLength > 0) {
            memcpy(localText, textBuffer.text, localLength);
            localText[localLength] = '\0';
        }
        xSemaphoreGive(displayMutexHandle);
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

    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        strncpy(localDisplay, calcDisplay, 21);
        localDisplay[21] = '\0';
        localOperator = lastOperator;
        localError = errorState;
        xSemaphoreGive(displayMutexHandle);
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

    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        localEmailIdx = emailIdx;
        if (localEmailIdx > 0) {
            memcpy(localEmailBuffer, emailBuffer, localEmailIdx);
            localEmailBuffer[localEmailIdx] = '\0';
        }
        xSemaphoreGive(displayMutexHandle);
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

    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(50)) == pdTRUE) {
        strncpy(localStatus, emailInfo.status, 31);
        localStatus[31] = '\0';
        strncpy(localRecipient, emailInfo.recipient, 63);
        localRecipient[63] = '\0';
        xSemaphoreGive(displayMutexHandle);
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
        uint8_t angle = frame * 45;
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
    if (xSemaphoreTake(uartMutexHandle, portMAX_DELAY) == pdTRUE) {
      xSemaphoreGive(uartMutexHandle);
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
              if (xSemaphoreTake(uartMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
                strncpy(ipEsp, ipStr, 19);
                ipEsp[19] = '\0';

                // Flash all LEDs
                HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_ORANGE_PIN | LED_BLUE_PIN | LED_RED_PIN, GPIO_PIN_SET);
                displayMode = DISPLAY_MODE_SYSTEM_INFO;
                textBuffer.needsUpdate = true;

                xSemaphoreGive(uartMutexHandle);
              }

              vTaskDelay(pdMS_TO_TICKS(200));
              HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_ORANGE_PIN | LED_BLUE_PIN | LED_RED_PIN, GPIO_PIN_RESET);
            }
          }
          else if (strncmp(buffer, "WIFI:", 5) == 0) {
                      char* statusStr = buffer + 5;

                      if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
                        strncpy(wifiSettings.status, statusStr, 31);
                        wifiSettings.status[31] = '\0';

                        if (displayMode == DISPLAY_MODE_SETTINGS) {
                          textBuffer.needsUpdate = true;
                        }

                        xSemaphoreGive(displayMutexHandle);

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
              if (xSemaphoreTake(uartMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
                strncpy(emailInfo.status, statusStr, 31);
                emailInfo.status[31] = '\0';

                if (displayMode != DISPLAY_MODE_EMAIL_STATUS) {
                  displayMode = DISPLAY_MODE_EMAIL_STATUS;
                }
                textBuffer.needsUpdate = true;

                xSemaphoreGive(uartMutexHandle);

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
  int cmdLen;

  if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
    strcpy(emailInfo.status, "Sending...");
    displayMode = DISPLAY_MODE_EMAIL_STATUS;
    xSemaphoreGive(displayMutexHandle);
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  cmdLen = snprintf(emailCmd, sizeof(emailCmd), "SEND_EMAIL:%s\n", emailInfo.recipient);

  if (xSemaphoreTake(uartMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
    HAL_UART_Transmit(&huart6, (uint8_t*)emailCmd, cmdLen, 1000);
    xSemaphoreGive(uartMutexHandle);
  }

  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
  vTaskDelay(pdMS_TO_TICKS(50));
  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
}

void KeyboardTask(void *argument) {
  uint8_t i, j;
  bool keyFound;
  static bool lastShiftState = false;
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
              if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(10)) == pdTRUE) {
                isEmailMode = emailMode;
                isCalcMode = (displayMode == DISPLAY_MODE_CALCULATOR);
                isSettingsMode = (displayMode == DISPLAY_MODE_SETTINGS);
                xSemaphoreGive(displayMutexHandle);
              }

              // Enter key handling
              if (keyCode == 0x28) {
                if (isCalcMode) {
                  CalculatorExecute();
                } else if (isEmailMode) {
                  c = '\n';
                  if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (emailIdx < 63) {
                      emailBuffer[emailIdx++] = c;
                    }
                    textBuffer.needsUpdate = true;
                    xSemaphoreGive(displayMutexHandle);
                  }
                } else if (isSettingsMode) {
                  if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    wifiSettings.editingSSID = !wifiSettings.editingSSID;
                    textBuffer.needsUpdate = true;
                    xSemaphoreGive(displayMutexHandle);
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
                  if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (emailIdx > 0) {
                      emailIdx--;
                      emailBuffer[emailIdx] = '\0';
                      textBuffer.needsUpdate = true;
                    }
                    xSemaphoreGive(displayMutexHandle);
                  }
                } else if (isSettingsMode) {
                  if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (wifiSettings.editingSSID && wifiSettings.ssidIdx > 0) {
                      wifiSettings.ssidIdx--;
                      wifiSettings.ssid[wifiSettings.ssidIdx] = '\0';
                      textBuffer.needsUpdate = true;
                    } else if (!wifiSettings.editingSSID && wifiSettings.passwordIdx > 0) {
                      wifiSettings.passwordIdx--;
                      wifiSettings.password[wifiSettings.passwordIdx] = '\0';
                      textBuffer.needsUpdate = true;
                    }
                    xSemaphoreGive(displayMutexHandle);
                  }
                } else {
                  if (textBuffer.length > 0) {
                    if (xSemaphoreTake(displayMutexHandle, portMAX_DELAY) == pdTRUE) {
                      textBuffer.length--;
                      textBuffer.text[textBuffer.length] = '\0';
                      textBuffer.needsUpdate = true;
                      xSemaphoreGive(displayMutexHandle);
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
                  if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (emailIdx < 63) {
                      emailBuffer[emailIdx++] = c;
                      textBuffer.needsUpdate = true;
                    }
                    xSemaphoreGive(displayMutexHandle);
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
                    if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
                      if (emailIdx < 63) {
                        emailBuffer[emailIdx++] = c;
                        textBuffer.needsUpdate = true;
                      }
                      xSemaphoreGive(displayMutexHandle);
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
        lastShiftState = currentShiftState;
      }
    } else {
      if (keyboardConnected) {
        keyboardConnected = false;
        memset(lastKeyState, 0, 6);
        lastShiftState = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void AddCharToBuffer(char c) {
  if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
    if (textBuffer.length < MAX_TEXT_LENGTH - 1) {
      textBuffer.text[textBuffer.length++] = c;
      textBuffer.text[textBuffer.length] = '\0';
      textBuffer.needsUpdate = true;
    }
    xSemaphoreGive(displayMutexHandle);
  }
}


void SendCharToESP(char c) {
  char keyMsg[15];

  snprintf(keyMsg, sizeof(keyMsg), "KEY:%c\n", c);

  if (xSemaphoreTake(uartMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
    HAL_UART_Transmit(&huart6, (uint8_t*)keyMsg, strlen(keyMsg), HAL_MAX_DELAY);
    xSemaphoreGive(uartMutexHandle);
  }
}

// Keep this function for external calls if needed
void ClearTextBuffer(void) {
  if (xSemaphoreTake(displayMutexHandle, pdMS_TO_TICKS(200)) == pdTRUE) {
    memset(textBuffer.text, 0, sizeof(textBuffer.text));
    textBuffer.length = 0;
    textBuffer.needsUpdate = true;
    xSemaphoreGive(displayMutexHandle);
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
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
