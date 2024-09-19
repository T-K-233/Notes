# Setting up ETH ANYbotics/elevation\_mapping on Ubuntu 20.04



## Install Dependencies

### Install grid\_map

```bash
sudo apt-get install ros-$ROS_DISTRO-grid-map
```

verify installation

```bash
roslaunch grid_map_demos simple_demo.launch
```



### Install Eigen

download [here](https://eigen.tuxfamily.org/index.php?title=Main\_Page).

```bash
export PATH="/home/tk/Documents/eigen-3.4.0/:$PATH"
```



### Install Kindr

```bash
cd ~/Desktop/catkin_workspace/src/
git clone git@github.com:anybotics/kindr.git
catkin build -w ~/Desktop/catkin_workspace/ kindr
```



### Install kindr\_ros

```bash
cd ~/Desktop/catkin_workspace/src/
git clone https://github.com/ANYbotics/kindr_ros.git
catkin build kindr_ros
```



### Install message\_logger

```bash
cd ~/Desktop/catkin_workspace/src/
git clone https://github.com/ANYbotics/message_logger.git
```



### Install elevation\_mapping

```bash
mkdir ~/Desktop/catkin_workspace/
```

```bash
mkdir src/
cd src/
git clone https://github.com/ANYbotics/elevation_mapping.git
cd ..
```



```bash
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
catkin build
```





## Running Example

```bash
sudo apt install ros-noetic-turtlebot3*
```



```bash
roslaunch elevation_mapping_demos turtlesim3_waffle_demo.launch
```



cavet:

it might not be able to find the launch file. In this case, clean the build and do

```
catkin build elevation_mapping_demos
```

to force it build and generate the install folder.

