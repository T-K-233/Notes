# Processing the SFU Motion Capture Dataset



## BVH Approach

Download .bvh animation data from the [SFU website](https://mocap.cs.sfu.ca/).



Enable the BVH import plugin in Blender



File -> Import -> Motion Capture (.bvh)

Use the following import settings

<figure><img src="../.gitbook/assets/image (6).png" alt=""><figcaption></figcaption></figure>

Autodesk MotionBuilder uses Z forward and Y up frame.

<figure><img src="../.gitbook/assets/image (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>



The animations are all in 120 FPS













## FBX Approach

> <mark style="color:red;">**Warning:**</mark> This approach does NOT work, since the fbx does not contain the original T-pose data.

Download .fbx animation data from the [SFU website](https://mocap.cs.sfu.ca/).



The .fbx file cannot be imported into Blender due to version mismatch.

<figure><img src="../.gitbook/assets/image (196).png" alt=""><figcaption></figcaption></figure>



Therefore, we need to convert the version using Autodesk MotionBuilder.



In MotionBuilder, open the .fbx file

<figure><img src="../.gitbook/assets/image (197).png" alt=""><figcaption></figcaption></figure>

Use default options to open

<figure><img src="../.gitbook/assets/image (198).png" alt=""><figcaption></figcaption></figure>



Drag through the timeline and verify that everything looks good

<figure><img src="../.gitbook/assets/image (199).png" alt=""><figcaption></figcaption></figure>



Click File -> Save As..., and overwrite the original .fbx file. Also use the default options to save.





Now the animation file should be able to be imported to Blender.



When importing to Blender, click the "Automatic Bone Orientation" option under "Armature" tab.

<figure><img src="../.gitbook/assets/image (2) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>









