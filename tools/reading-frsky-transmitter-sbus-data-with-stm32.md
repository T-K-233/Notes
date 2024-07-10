# Reading FrSky Transmitter SBUS data with STM32







<figure><img src="../.gitbook/assets/image (9).png" alt=""><figcaption></figcaption></figure>



```c

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHANNEL_VALUE_HI        1811.f
#define CHANNEL_VALUE_LO        172.f
#define CHANNEL_VALUE_OFFSET    (0.5 * (CHANNEL_VALUE_HI + CHANNEL_VALUE_LO))
#define CHANNEL_VALUE_SCALE     (2000.f / (CHANNEL_VALUE_HI - CHANNEL_VALUE_LO) + 0.5f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static inline int16_t clamp(int16_t value, int16_t min, int16_t max) {
  return (value > max) ? max : ((value < min) ? min : value);
}

static inline float clampf(float value, float min, float max) {
  return (value > max) ? max : ((value < min) ? min : value);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t channels_raw[20];
int16_t channels[20];
uint8_t rx_buffer[32];

/**
 * Remap raw channel value to range of [-1000, 1000]
 */
int16_t remap(uint16_t raw) {
  int16_t output = (int16_t)(((float)raw - CHANNEL_VALUE_OFFSET) * CHANNEL_VALUE_SCALE);
  output = clamp(output, -1000, 1000);
  return output;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (rx_buffer[0] != 0x0F || rx_buffer[24] != 0x00) {
    // incorrect SBUS header and footer

    HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, 25);
    return;
  }


  // 16 servo channels
  channels_raw[0] = (uint16_t)(*(uint32_t *)(rx_buffer + 1)) & 0x7FF;
  channels_raw[1] = (uint16_t)(*(uint32_t *)(rx_buffer + 2) >> 3) & 0x7FF;
  channels_raw[2] = (uint16_t)(*(uint32_t *)(rx_buffer + 3) >> 6) & 0x7FF;
  channels_raw[3] = (uint16_t)(*(uint32_t *)(rx_buffer + 5) >> 1) & 0x7FF;
  channels_raw[4] = (uint16_t)(*(uint32_t *)(rx_buffer + 6) >> 4) & 0x7FF;
  channels_raw[5] = (uint16_t)(*(uint32_t *)(rx_buffer + 7) >> 7) & 0x7FF;
  channels_raw[6] = (uint16_t)(*(uint32_t *)(rx_buffer + 9) >> 2) & 0x7FF;
  channels_raw[7] = (uint16_t)(*(uint32_t *)(rx_buffer + 10) >> 5) & 0x7FF;
  channels_raw[8] = (uint16_t)(*(uint32_t *)(rx_buffer + 12) >> 0) & 0x7FF;
  channels_raw[9] = (uint16_t)(*(uint32_t *)(rx_buffer + 13) >> 3) & 0x7FF;
  channels_raw[10] = (uint16_t)(*(uint32_t *)(rx_buffer + 14) >> 6) & 0x7FF;
  channels_raw[11] = (uint16_t)(*(uint32_t *)(rx_buffer + 16) >> 1) & 0x7FF;
  channels_raw[12] = (uint16_t)(*(uint32_t *)(rx_buffer + 17) >> 4) & 0x7FF;
  channels_raw[13] = (uint16_t)(*(uint32_t *)(rx_buffer + 18) >> 7) & 0x7FF;
  channels_raw[14] = (uint16_t)(*(uint32_t *)(rx_buffer + 20) >> 2) & 0x7FF;
  channels_raw[15] = (uint16_t)(*(uint32_t *)(rx_buffer + 21) >> 5) & 0x7FF;

  // additional channels
  channels_raw[16] = (rx_buffer[23] & 0x01);
  channels_raw[17] = (rx_buffer[23] & 0x02);
  uint8_t frame_lost = (rx_buffer[23] & 0x04);
  uint8_t failsafe_activated = (rx_buffer[23] & 0x08);

  channels[0] = remap(channels_raw[0]);
  channels[1] = remap(channels_raw[1]);
  channels[2] = remap(channels_raw[2]);
  channels[3] = remap(channels_raw[3]);
  channels[4] = remap(channels_raw[4]);
  channels[5] = remap(channels_raw[5]);
  channels[6] = remap(channels_raw[6]);
  channels[7] = remap(channels_raw[7]);
  channels[8] = remap(channels_raw[8]);
  channels[9] = remap(channels_raw[9]);
  channels[10] = remap(channels_raw[10]);
  channels[11] = remap(channels_raw[11]);
  channels[12] = remap(channels_raw[12]);
  channels[13] = remap(channels_raw[13]);
  channels[14] = remap(channels_raw[14]);
  channels[15] = remap(channels_raw[15]);
//  channels[16] = remap(channels_raw[16]);
//  channels[17] = remap(channels_raw[17]);

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, 25);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */



  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, 25);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t rx_data;
    HAL_StatusTypeDef rx_status = HAL_UART_Receive(&huart2, &rx_data, 1, 100);

    if (rx_status == HAL_OK && rx_data == '\n') {
      char str[64];
      sprintf(str, "ch1:%d\tch2:%d\tch3:%d\tch4:%d\tch5:%d\tch6:%d\t", channels[0], channels[1], channels[2], channels[3], channels[4], channels[5]);
      HAL_UART_Transmit(&huart2, str, strlen(str), 100);
      sprintf(str, "ch7:%d\tch8:%d\tch9:%d\tch10:%d\tch11:%d\tch12:%d\t", channels[6], channels[7], channels[8], channels[9], channels[10], channels[11]);
      HAL_UART_Transmit(&huart2, str, strlen(str), 100);
      sprintf(str, "ch13:%d\tch14:%d\tch15:%d\tch16:%d\tch17:%d\tch18:%d\n", channels[12], channels[13], channels[14], channels[15], channels[16], channels[17]);
      HAL_UART_Transmit(&huart2, str, strlen(str), 100);
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```







