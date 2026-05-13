# Setting up ROS 2 Jazzy on Ubuntu

## Environment

Ubuntu 24.04

{% embed url="https://docs.ros.org/en/jazzy/Installation.html" %}

### Setup Locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```



### Setup Sources

Enable the Ubuntu Universe repository.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Install dev tools.

```bash
sudo apt update && sudo apt install ros-dev-tools
```



### Install ROS 2 packages

> **WARNING**: ROS2 tools like colcon does not quite work with `conda` environments, so it's recommended to install ROS-related things directly under system environment. In this case, make sure when running the commands, the conda environment is disabled:
>
> ```bash
> > (base)$ conda deactivate
> > $
> ```
>
> `apt` will install things to system directory, but `pip` will install it to current active Python directory.

Update apt repository caches after setting up the repositories, and ensure the system is up to date before installing new packages.

```bash
sudo apt update
sudo apt upgrade
```

Install Desktop Install, including ROS, RViz, demos, and tutorials.

```bash
sudo apt install ros-jazzy-desktop
```



## Install ROS2 Control

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```



