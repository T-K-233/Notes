# Setting up ROS 2 Humble Hawksbill on Ubuntu

## Environment

Ubuntu 22.02

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

other possible required dependencies

```bash
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
```

### Install Colcon

Colcon is a tool used to build ROS 2 software packages.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```



With Ubuntu 22.04 + ROS 2 Humble combination, we also need the following dependencies for building turtlesim:

```bash
pip install empy
pip install lark
```

Colcon also has a tool, `colcon_cd`, to help navigating to package directories.

We need to source the script to use it ([reference](https://colcon.readthedocs.io/en/released/user/installation.html?highlight=colcon\_cd#quick-directory-changes)), so either source it or add it to `~/.bashrc`

```bash
source /usr/share/colcon_cd/function/colcon_cd.sh
```

