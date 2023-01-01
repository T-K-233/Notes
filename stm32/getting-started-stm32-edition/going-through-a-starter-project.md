# Going Through A Starter Project

## 0. Create New Project

**File** -> **New** -> **STM32 Project**

****

Identify the MCU version that you are using. For this entire tutorial, we will be using STM32F446RET6 unless otherwise noticed.

<details>

<summary>Identifying the MCU version</summary>



</details>

<figure><img src="../../.gitbook/assets/image (15).png" alt=""><figcaption></figcaption></figure>



Search for "STM32F446RET6" and select the first result. Then click **Next**.

<figure><img src="../../.gitbook/assets/image (22).png" alt=""><figcaption></figcaption></figure>



Name the project, select a suitable location, and then keep the following settings. Then click **Finish**.

<figure><img src="../../.gitbook/assets/image (14).png" alt=""><figcaption></figcaption></figure>



## 1. Configure STM32

STM32CubeIDE will open the .ioc graphical configuration utility first.

In the left sidebar, select **System Core** -> **SYS**.

Select Debug to use Serial Wire

![](<../../.gitbook/assets/image (113).png>)



Select **Connectivity** -> **USART2**.

Set **Mode** to be Asynchronous.

![](<../../.gitbook/assets/image (26) (1).png>)



In the Clock Configuration tab, set **HCLK** to **160 MHz** and click Enter, the IDE will automatically derive the required PLL parameters.

![](<../../.gitbook/assets/image (76).png>)



Press **Ctrl+S**. STM32CubeIDE will start to generate the codes.



## 2. Code with Minimal Setup

In `main.h` and `main.c`, add the following code

{% tabs %}
{% tab title="Code" %}
#### main.h

<pre class="language-c"><code class="lang-c"><strong>/* Private includes ----------------------------------------------------------*/
</strong>/* USER CODE BEGIN Includes */
#include &#x3C;stdio.h>
#include &#x3C;string.h>
/* USER CODE END Includes */
</code></pre>

####

#### main.c

```c
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint32_t counter = 0;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    char str[64];

    sprintf(str, "hello world %lu\r\n", counter);

    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);

    counter += 1;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

```
{% endtab %}

{% tab title="Screenshot" %}
#### main.h

<figure><img src="../../.gitbook/assets/image (54).png" alt=""><figcaption></figcaption></figure>

#### main.c

<figure><img src="../../.gitbook/assets/image (7).png" alt=""><figcaption></figcaption></figure>
{% endtab %}
{% endtabs %}



## 3. Upload

Click the green "run ..." button

<figure><img src="../../.gitbook/assets/image (18).png" alt=""><figcaption></figcaption></figure>

If there's a popup menu, click "OK".

<figure><img src="../../.gitbook/assets/image (19) (2).png" alt=""><figcaption></figcaption></figure>



## 4. Result

We should be able to see hello world with an upward counter printing from any serial monitor.

Recommend using the [VSCode Serial Monitor extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor) from Microsoft.



## 5. Code with Cleaner Hierarchy

Although our code is running now, putting everything into `main.c` and `main.h` isn't really a good idea ---- these files are autogenerated by STM32CubeIDE, and we would risk breaking it when putting and editing our code between those autogenerated lines.&#x20;

Thus, a better way of managing the code would be to separate the system-managed code from our user code. Now we would create two new source files, `app.h` and `app.c`, and move our existing code there.





In the Project Explorer, right click the "Src" folder, select "New -> Source File".

<figure><img src="../../.gitbook/assets/image (21) (2).png" alt=""><figcaption></figcaption></figure>

Name the file "app.c" and click "Finish".

<figure><img src="../../.gitbook/assets/image (16) (2).png" alt=""><figcaption></figcaption></figure>



Similarly, in the Project Explorer, right click the "Inc" folder, select "New -> Header File".

<figure><img src="../../.gitbook/assets/image (1) (5).png" alt=""><figcaption></figcaption></figure>



Name the file "app.h" and click "Finish".

<figure><img src="../../.gitbook/assets/image (4) (6) (1).png" alt=""><figcaption></figcaption></figure>

Then, we move all the previous user codes into these newly created files.

{% tabs %}
{% tab title="Code" %}
#### app.h

```c
/*
 * app.h
 *
 *  Created on: Dec 30, 2022
 *      Author: TK
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdio.h>
#include <string.h>

#include "stm32g4xx_hal.h"


void APP_init();

void APP_main();

#endif /* INC_APP_H_ */

```



#### app.c

```c
/*
 * app.c
 *
 *  Created on: Dec 30, 2022
 *      Author: TK
 */

#include "app.h"

uint32_t counter = 0;

void APP_init() {

}

void APP_main() {
  char str[64];

  sprintf(str, "hello world %lu\r\n", counter);

  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);

  counter += 1;
}

```
{% endtab %}

{% tab title="Screenshot" %}

{% endtab %}
{% endtabs %}



Finally, we need to invoke the user application functions in `main.c`

{% tabs %}
{% tab title="Code" %}
#### main.h

```c
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

```



#### main.c

```c

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  APP_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    APP_main();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

```
{% endtab %}

{% tab title="Screenshot" %}

{% endtab %}
{% endtabs %}








