# ROS 2 Issue in Ubuntu with conda

It is common to see the following error, where when building a ROS package, it uses the wrong python executable, and then fails to find ros Python libraries.

```bash
colcon build
Starting >>> humanoid_v2
--- stderr: humanoid_v2                         
Traceback (most recent call last):
  File "/opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 22, in <module>
    from catkin_pkg.package import parse_package_string
ModuleNotFoundError: No module named 'catkin_pkg'
```

Meanwhile, the python3 in the system terminal still points to the correct location.

```bash
python3
Python 3.10.12 (main, Jun 11 2023, 05:26:28) [GCC 11.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> from catkin_pkg.package import parse_package_string

```



Solution:

This might because that the ROS package is created inside a conda environment. Try to delete the following folders

```bash
rm -rf ./build/
rm -rf ./install/
rm -rf ./log/
```

and then exit conda to the root environment

```bash
(base) $ conda deactivate
$
```



and then build the package again.

