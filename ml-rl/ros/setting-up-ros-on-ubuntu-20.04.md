# Setting up ROS on Ubuntu 20.04

## Environment

Ubuntu 20.04

{% embed url="https://wiki.ros.org/noetic/Installation/Ubuntu" %}

### Setup Sources <a href="#installation.2fubuntu.2fsources.setup_your_sources.list" id="installation.2fubuntu.2fsources.setup_your_sources.list"></a>

First install curl

{% code overflow="wrap" %}
```bash
sudo apt update && sudo apt install curl
```
{% endcode %}

{% code overflow="wrap" %}
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
{% endcode %}

Add the ROS key with apt.

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
sudo apt install ros-noetic-desktop
```



To install support for 2D/3D visualization tools

```bash
sudo apt install ros-noetic-desktop-full
```



### Environment setup <a href="#noetic.2finstallation.2fdebenvironment.environment_setup" id="noetic.2finstallation.2fdebenvironment.environment_setup"></a>

Put this line in bashrc

<pre class="language-bash" data-title="~/.bashrc"><code class="lang-bash">...
<strong># ROS Noetic
</strong>source /opt/ros/noetic/setup.bash
...
</code></pre>



### Install dependencies for building packages

{% code overflow="wrap" %}
```bash
sudo apt install build-essential
sudo apt install python3-pip
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
```
{% endcode %}



## Install catkin

{% code overflow="wrap" %}
```bash
sudo apt install python3-catkin-lint python3-catkin-tools
```
{% endcode %}

