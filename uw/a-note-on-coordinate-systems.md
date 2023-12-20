# A Note on Coordinate Systems

## Different Coordinate Frames

### Standardized

<table><thead><tr><th>Translation</th><th width="182">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:red;">X</mark></td><td>Pitch</td><td><mark style="color:red;">X</mark></td></tr><tr><td>Front</td><td><mark style="color:green;">Y</mark></td><td>Roll</td><td><mark style="color:green;">Y</mark></td></tr><tr><td>Up</td><td><mark style="color:blue;">Z</mark></td><td>Yaw</td><td><mark style="color:blue;">Z</mark></td></tr><tr><td></td><td></td><td>(QW, QX, QY, QZ)</td><td></td></tr></tbody></table>



### Blender

[Blender Manual - Axes](a-note-on-coordinate-systems.md#blender)

<table><thead><tr><th>Translation</th><th width="184">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:red;">X</mark></td><td>Pitch</td><td><mark style="color:red;">X</mark></td></tr><tr><td>Front</td><td><mark style="color:green;">Y</mark></td><td>Roll</td><td><mark style="color:green;">Y</mark></td></tr><tr><td>Up</td><td><mark style="color:blue;">Z</mark></td><td>Yaw</td><td><mark style="color:blue;">Z</mark></td></tr><tr><td></td><td></td><td>(QW, QX, QY, QZ)</td><td></td></tr></tbody></table>



### OnShape

<table><thead><tr><th>Translation</th><th width="186">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:red;">X</mark></td><td>Pitch</td><td><mark style="color:red;">X</mark></td></tr><tr><td>Front</td><td><mark style="color:green;">Y</mark></td><td>Roll</td><td><mark style="color:green;">Y</mark></td></tr><tr><td>Up</td><td><mark style="color:blue;">Z</mark></td><td>Yaw</td><td><mark style="color:blue;">Z</mark></td></tr></tbody></table>



### NVIDIA PhysX (Isaac Gym, Isaac Sim, Omniverse)

<table><thead><tr><th>Translation</th><th width="186">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:green;">-Y</mark></td><td>Pitch</td><td><mark style="color:green;">-Y</mark></td></tr><tr><td>Front</td><td><mark style="color:red;">X</mark></td><td>Roll</td><td><mark style="color:red;">X</mark></td></tr><tr><td>Up</td><td><mark style="color:blue;">Z</mark></td><td>Yaw</td><td><mark style="color:blue;">Z</mark></td></tr><tr><td></td><td></td><td>(QX, QY, QZ, QW)</td><td></td></tr></tbody></table>



### glTF

[glTF coordinate systems and units](a-note-on-coordinate-systems.md#different-coordinate-frames)

<table><thead><tr><th>Translation</th><th width="186">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:red;">-X</mark></td><td>Pitch</td><td><mark style="color:red;">-X</mark></td></tr><tr><td>Front</td><td><mark style="color:blue;">Z</mark></td><td>Roll</td><td><mark style="color:blue;">Z</mark></td></tr><tr><td>Up</td><td><mark style="color:green;">Y</mark></td><td>Yaw</td><td><mark style="color:green;">Y</mark></td></tr></tbody></table>



### MJCF

[MJCF Frame Orientations](https://mujoco.readthedocs.io/en/latest/modeling.html#frame-orientations)

(QW, QX, QY, QZ)



### Autodesk MotionBuilder

<table><thead><tr><th>Translation</th><th width="186">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:red;">-X</mark></td><td>Pitch</td><td><mark style="color:red;">-X</mark></td></tr><tr><td>Front</td><td><mark style="color:blue;">Z</mark></td><td>Roll</td><td><mark style="color:blue;">Z</mark></td></tr><tr><td>Up</td><td><mark style="color:green;">Y</mark></td><td>Yaw</td><td><mark style="color:green;">Y</mark></td></tr></tbody></table>





### ROS

[ROS Coordinate Frame Conventions](https://www.ros.org/reps/rep-0103.html#id19)

<table><thead><tr><th>Translation</th><th width="186">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:green;">-Y</mark></td><td>Pitch</td><td><mark style="color:green;">-Y</mark></td></tr><tr><td>Front</td><td><mark style="color:red;">X</mark></td><td>Roll</td><td><mark style="color:red;">X</mark></td></tr><tr><td>Up</td><td><mark style="color:blue;">Z</mark></td><td>Yaw</td><td><mark style="color:blue;">Z</mark></td></tr></tbody></table>



### SteamVR

<table><thead><tr><th>Translation</th><th width="184">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:green;">-Y</mark></td><td>Pitch</td><td></td></tr><tr><td>Front</td><td><mark style="color:red;">X</mark></td><td>Roll</td><td></td></tr><tr><td>Up</td><td><mark style="color:blue;">Z</mark></td><td>Yaw</td><td></td></tr></tbody></table>



### Unreal Engine

<table><thead><tr><th>Translation</th><th width="188">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:green;">Y</mark></td><td></td><td></td></tr><tr><td>Front</td><td><mark style="color:red;">X</mark></td><td></td><td></td></tr><tr><td>Up</td><td><mark style="color:blue;">Z</mark></td><td></td><td></td></tr></tbody></table>

> **Note**: UE uses the left-hand coordinate system.



### CrazyFlie

[The Coordinate System of the CrazyFlie 2.X](https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/)

<table><thead><tr><th>Translation</th><th width="182">Axis</th><th>Rotation</th><th>Axis</th></tr></thead><tbody><tr><td>Right</td><td><mark style="color:green;">-Y</mark></td><td>Pitch</td><td><mark style="color:green;">Y</mark> (left-hand)</td></tr><tr><td>Front</td><td><mark style="color:red;">X</mark></td><td>Roll</td><td><mark style="color:red;">X</mark> (right-hand)</td></tr><tr><td>Up</td><td><mark style="color:blue;">Z</mark></td><td>Yaw</td><td><mark style="color:blue;">Z</mark> (right-hand)</td></tr></tbody></table>



## Conversion Between Frames

From SteamVR (LiveLink) to Unreal:

<figure><img src="../.gitbook/assets/image (9) (2) (2).png" alt=""><figcaption></figcaption></figure>
