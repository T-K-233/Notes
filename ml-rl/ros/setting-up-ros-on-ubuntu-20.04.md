# Setting up ROS on Ubuntu 20.04



{% embed url="https://wiki.ros.org/noetic/Installation/Ubuntu" %}

### Setup your sources.list <a href="#installation.2fubuntu.2fsources.setup_your_sources.list" id="installation.2fubuntu.2fsources.setup_your_sources.list"></a>

{% code overflow="wrap" %}
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
{% endcode %}

### Set up your keys <a href="#installation.2fubuntu.2fsources.set_up_your_keys" id="installation.2fubuntu.2fsources.set_up_your_keys"></a>

{% code overflow="wrap" %}
```bash
sudo apt install curl # if you haven't already installed curl
```
{% endcode %}

{% code overflow="wrap" %}
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
{% endcode %}

### Installation <a href="#installation-1" id="installation-1"></a>

```bash
sudo apt update
```

```bash
sudo apt install ros-noetic-desktop-full
```



### Environment setup <a href="#noetic.2finstallation.2fdebenvironment.environment_setup" id="noetic.2finstallation.2fdebenvironment.environment_setup"></a>

Put this line in bashrc

{% code title="bash.rc" %}
```bash
...
# ROS Noetic
source /opt/ros/noetic/setup.bash
...
```
{% endcode %}



### Install dependencies for building packages

{% code overflow="wrap" %}
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
{% endcode %}



```bash
sudo apt install python3-rosdep
```



## Install catkin

{% code overflow="wrap" %}
```bash
sudo apt install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
```
{% endcode %}
