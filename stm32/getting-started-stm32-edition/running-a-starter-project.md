# Running A Starter Project

## 0. Create New Project

**File** -> **New** -> **STM32 Project**

![](<../../.gitbook/assets/image (32).png>)



Search "STM32F446RET6" and select the first result. Then click **Next**.

![](<../../.gitbook/assets/image (62).png>)



Name the project, select a suitable location, and then keep the following settings. Then click **Finish**.

![](<../../.gitbook/assets/image (44).png>)



## 1. Configure STM32

STM32CubeIDE will open the .ioc graphical configuration utility first.

In the left sidebar, select **System Core** -> **SYS**.

Select Debug to use Serial Wire

![](<../../.gitbook/assets/image (113).png>)



Select **Connectivity** -> **USART2**.

Set **Mode** to be Asynchronous.

![](<../../.gitbook/assets/image (26).png>)



In the Clock Configuration tab, set **HCLK** to **160 MHz** and click Enter, the IDE will automatically derive the required PLL parameters.

![](<../../.gitbook/assets/image (76).png>)



Press **Ctrl+S**. STM32CubeIDE will start to generate the codes.



## 2. Code

In `main.h`, add the following code

![](<../../.gitbook/assets/image (2) (1) (1).png>)



In `main.c`, add the following code

![](<../../.gitbook/assets/image (99).png>)



## 3. Upload

Click the green run ... button

![](<../../.gitbook/assets/image (67).png>)

If there's a popup menu, click Run.



## 4. Result

We should be able to see hello world with a upward counter printing from any serial monitor.

Recommended serial monitor is the [VSCode Serial Monitor extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor) from Microsoft.

![](<../../.gitbook/assets/image (122).png>)

