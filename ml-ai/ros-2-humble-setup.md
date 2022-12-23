# ROS 2 (Humble) Setup

{% embed url="https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html" %}

### Setup Sources

Enable the Ubuntu Universe repository.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```



Add the ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```



Add the repository to your sources list.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```



### Install ROS 2 packages

Update apt repository caches after setting up the repositories, and ensure the system is up to date before installing new packages.

```bash
sudo apt update
sudo apt upgrade
```



Install Desktop Install, including ROS, RViz, demos, and tutorials.

```bash
sudo apt install ros-humble-desktop
```



### Sourcing

```bash
source /opt/ros/humble/setup.bash
```

or

```bash
source /opt/ros/humble/setup.sh
```

or

```bash
source /opt/ros/humble/setup.zsh
```



## Additional Tools

```bash
sudo apt install python3-rosdep2
rosdep update
```







