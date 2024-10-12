# Changing STM32CubeIDE Settings

## Set Code Style

Goto **Window** -> **Preferences**

![](<../../.gitbook/assets/image (95).png>)

Goto **C/C++** -> **Formatter**, click the **Import** button.

![](<../../.gitbook/assets/image (91).png>)



{% file src="../../.gitbook/assets/Deuncertainfy_C_Cpp_Format_Config_STM32CubeIDE.xml" %}



In the popup window, select the formatter config file

![](<../../.gitbook/assets/image (134).png>)

Then, in the Name Style tab, change the following fields:

Variable: Lower Case, "\_" as delimiter

![](<../../.gitbook/assets/image (54).png>)

Class Field: Lower Case, "\_" as delimiter

![](<../../.gitbook/assets/image (74).png>)

Click **Apply and Close** button

![](<../../.gitbook/assets/image (92).png>)

## Auto Format Code

Right-click on any file that needs to be formatted. Select **Source** -> **Format**.

![](<../../.gitbook/assets/image (22) (1) (1).png>)

## Change to Dark Mode

Goto **Window** -> **Preferences** -> **General** -> **Appearance**

Select **Dark** mode under Theme

![](<../../.gitbook/assets/image (55).png>)

## Open Serial Monitor

At the bottom panel, click **Open Console** button -> **3 Command Shell Console**.

![](<../../.gitbook/assets/image (64).png>)

Select "Serial Port", and click **New** button.

![](<../../.gitbook/assets/image (35).png>)

Select the correct COM port. Click **Finish**.

![](<../../.gitbook/assets/image (3) (1) (1) (1) (1) (1) (1) (1) (1).png>)

Change Encoding to "UTF-8". Click **Ok**.

![](<../../.gitbook/assets/image (27) (1) (1).png>)

## Enable Floating Point printf()

Goto **Project** -> **Properties**

![](<../../.gitbook/assets/image (66).png>)

Goto **C/C++ Build** -> **Settings** -> **MCU Settings**, check the "Use float with printf from newlib-nano" setting.

![](<../../.gitbook/assets/image (43) (1).png>)
