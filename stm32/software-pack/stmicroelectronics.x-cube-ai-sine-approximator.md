# STMicroelectronics.X-CUBE-AI - Sine Approximator

## Preparing Model

See this [Google Colab Notebook](https://colab.research.google.com/drive/1S3m5H5iMvbBAEL62\_QmndETPe8HDsVba?usp=sharing).



## STM32CubeIDE Setup

Go to Help -> Manage Embedded Software Packages

<figure><img src="../../.gitbook/assets/image (148).png" alt=""><figcaption></figcaption></figure>

Install the latest Artificial Intelligence package under STMicroelectronics -> X-CUBE-AI tab

<figure><img src="../../.gitbook/assets/image (60).png" alt=""><figcaption></figcaption></figure>

Create a new STM32 project. To see list only the MCU supported by the AI package, check the Enable checkbox under MIDDLEWARE -> Artificial Intelligence in the project selector.

STM32F446RET6 is supported.

<figure><img src="../../.gitbook/assets/image (150).png" alt=""><figcaption></figcaption></figure>

After creating the project, configure the System Core and UART as normal. Then, in the ioc configuration page, select Software Packs -> Select Components

<figure><img src="../../.gitbook/assets/image (147).png" alt=""><figcaption></figcaption></figure>

Enable the Artificial Intelligence pack, and select Application as Application Template

<figure><img src="../../.gitbook/assets/image (155).png" alt=""><figcaption></figcaption></figure>



After enabling the pack, there will be a new configurable field STMicroelectronics.X-CUBE-AI under Software Packs.

When clicking it, it will prompt the following message if the clock is in default setting. Accept the change, as it will set the system clock to the highest frequency (180MHz for STM32F446).

<figure><img src="../../.gitbook/assets/image (153).png" alt=""><figcaption></figcaption></figure>



Click "Add Network", set the model name to desired name, model type to "TFLite", and load the .tflite model file we exported in the Jupyter Notebook. Finally, click Analyze.

<figure><img src="../../.gitbook/assets/image (154).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (6).png" alt=""><figcaption></figcaption></figure>

We can click the "Show graph" button to visualize the model

<figure><img src="../../.gitbook/assets/image (149).png" alt=""><figcaption></figcaption></figure>

Generate code by saving the .ioc file.

The AI pack related files will be put under "X-CUBE-AI" folder.

<figure><img src="../../.gitbook/assets/image (151).png" alt=""><figcaption></figcaption></figure>



Our model weights are loaded in the `sine_model_data_params.c` file.

<figure><img src="../../.gitbook/assets/image (152).png" alt=""><figcaption></figcaption></figure>



Because we do not have printf redirected, we need to change the printf lines in the generated file to sprintf and UART\_Transmit:

Before:

```c
void MX_X_CUBE_AI_Init(void)
{
    /* USER CODE BEGIN 5 */
  printf("\r\nTEMPLATE - initialization\r\n");

  ai_boostrap(data_activations0);
    /* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */
  int res = -1;

  printf("TEMPLATE - run - main loop\r\n");

  ...
}

static void ai_log_err(const ai_error err, const char *fct)
{
  /* USER CODE BEGIN log */
  if (fct)
    printf("TEMPLATE - Error (%s) - type=0x%02x code=0x%02x\r\n", fct,
        err.type, err.code);
  else
    printf("TEMPLATE - Error - type=0x%02x code=0x%02x\r\n", err.type, err.code);

  do {} while (1);
  /* USER CODE END log */
}

```

After:

```c
void MX_X_CUBE_AI_Init(void)
{
    /* USER CODE BEGIN 5 */
  char str[64];
  sprintf(str, "\r\nTEMPLATE - initialization\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);

  ai_boostrap(data_activations0);
    /* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */
  int res = -1;

  char str[64];
  sprintf(str, "TEMPLATE - run - main loop\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
  ...
}

static void ai_log_err(const ai_error err, const char *fct)
{
  /* USER CODE BEGIN log */
  if (fct) {
    char str[64];
    sprintf(str, "TEMPLATE - Error (%s) - type=0x%02x code=0x%02x\r\n", fct,
        err.type, err.code);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
  }
  else {
    char str[64];
    sprintf(str, "TEMPLATE - Error - type=0x%02x code=0x%02x\r\n", err.type, err.code);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
  }
  do {} while (1);
  /* USER CODE END log */
}

```



And also grab the huart definition from main with extern

```c
/* USER CODE BEGIN includes */
extern UART_HandleTypeDef huart2;
/* USER CODE END includes */
```



## User Code

We will put our user code in the `acquire_and_process_data` and `post_process` function. For demo, we will just use a constant 2.0 as the input to the neural network, and see if the result is close to `sin(2.0) = 0.9092974268256817`.



```c
int acquire_and_process_data(ai_i8* data[]) {
  /* fill the inputs of the c-model
  for (int idx=0; idx < AI_SINE_MODEL_IN_NUM; idx++ )
  {
      data[idx] = ....
  }
  */
  // set input to 2.0
  *((ai_float *)data[0]) = 2.0f;
  return 0;
}

int post_process(ai_i8* data[]) {
  // print output
  float y_predicted = *((float *)data[0]);

  char str[64];
  sprintf(str, "result: %f\r\n", y_predicted);
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
  return 0;
}

```



Running the code, we get the following output.

<figure><img src="../../.gitbook/assets/image (156).png" alt=""><figcaption></figcaption></figure>



