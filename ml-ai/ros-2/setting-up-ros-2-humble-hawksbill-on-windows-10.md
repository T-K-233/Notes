# Setting up ROS 2 Humble Hawksbill on Windows 10

## Environment

Windows 10



{% embed url="https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html" %}

### Install Chocolatey

Run PowerShell as admin

<figure><img src="../../.gitbook/assets/image (4) (3).png" alt=""><figcaption></figcaption></figure>

Ensure Get-ExecutionPolicy is not Restricted

```powershell
PS C:\Windows\system32>  Get-ExecutionPolicy
Restricted
```

To change this, run

```powershell
PS C:\Windows\system32> Set-ExecutionPolicy AllSigned

Execution Policy Change
The execution policy helps protect you from scripts that you do not trust. Changing the execution policy might expose
you to the security risks described in the about_Execution_Policies help topic at
https:/go.microsoft.com/fwlink/?LinkID=135170. Do you want to change the execution policy?
[Y] Yes  [A] Yes to All  [N] No  [L] No to All  [S] Suspend  [?] Help (default is "N"): Y
```

```powershell
PS C:\Windows\system32> Get-ExecutionPolicy
AllSigned
```

Set the installation directory:

```powershell
$env:ChocolateyInstall = "D:\Documents\chocolately"
```



```powershell
PS C:\Windows\system32> Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
```

<figure><img src="../../.gitbook/assets/image (2) (1) (2).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (5) (1) (3).png" alt=""><figcaption></figcaption></figure>



### Install Python and VCredist140

```powershell
choco install -y python --version 3.8.3
choco install -y vcredist2013 vcredist140
```





### Install OpenSSL

Download the _Win64 OpenSSL v1.1.1n_ OpenSSL installer from [this page](https://slproweb.com/products/Win32OpenSSL.html). Scroll to the bottom of the page and download _Win64 OpenSSL v1.1.1n_. Don’t download the Win32 or Light versions, or the v3.X.Y installers.

```powershell
setx /m OPENSSL_CONF "D:\Documents\OpenSSL-Win64\bin\openssl.cfg"
```

Add openssl to PATH

`D:\Documents\OpenSSL-Win64\bin`



### Install Visual Studio

An easy way to make sure they’re installed is to select the `Desktop development with C++` workflow during the install.



### Install OpenCV

Download a precompiled version of OpenCV 3.4.6 from [https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip](https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip) .



```powershell
setx /m OpenCV_DIR "D:\Documents\ros2-humble\tools\opencv"
```

Add `D:\Documents\ros2-humble\tools\opencv\x64\vc16\bin` to path.



### Install other dependencies

```
choco install -y cmake
```

Add `C:\Program Files\CMake\bin` to path



```
choco install -y -s D:\Downloads asio cunit eigen tinyxml-usestl tinyxml2 bullet
```

```
python -m pip install -U pip setuptools==59.6.0
```

```
python -m pip install -U catkin_pkg cryptography empy importlib-metadata lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pyyaml rosdistro
```



### Install Qt5

```powershell
setx /m Qt5_DIR D:\Documents\Qt\Qt5.12.12\5.12.12\msvc2017_64
setx /m QT_QPA_PLATFORM_PLUGIN_PATH D:\Documents\Qt\Qt5.12.12\5.12.12\msvc2017_64\plugins\platforms
```

###

### Install Graphviz

To run rqt\_graph you need to [download](https://graphviz.gitlab.io/\_pages/Download/Download\_windows.html) and install [Graphviz](https://graphviz.gitlab.io/).&#x20;

Select add to PATH in the installer options

D:\Documents\ros2-windows\tools\Graphviz







### Install ROS2

```bash
call D:\Documents\ros2-humble\local_setup.bat
```





[https://www.reddit.com/r/ROS/comments/b7jsgx/trouble\_installing\_ros\_2\_on\_windows/](https://www.reddit.com/r/ROS/comments/b7jsgx/trouble\_installing\_ros\_2\_on\_windows/)



[https://answers.ros.org/question/379164/rti-connext-dds-environment-script-not-found-ros2-on-windows/](https://answers.ros.org/question/379164/rti-connext-dds-environment-script-not-found-ros2-on-windows/)

[https://answers.ros.org/question/294209/ros2-command-not-found-in-windows/](https://answers.ros.org/question/294209/ros2-command-not-found-in-windows/)



Note: need to use cmd.exe, powershell will not work.



### Install Fast DDS

Download the latest windows release from here:

[https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-dds](https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-dds)





### Install RTI Connext DDS





```
call "D:\Documents\ros2-humble\tools\rti_context_dds\rti_connext_dds-6.1.1\resource\scripts\rtisetenv_x64Win64VS2017.bat"
```

