# Repeat an Animation with Offset in Blender



Many game animation asset motions comes with looped animations. Typically they provide separate file s for the start of animation, end of animation, and a repeatable loop body of the animation.

In this note, we will see how to extend and repeat this animation indefinitely in blender.



### Example File

<figure><img src="../.gitbook/assets/image (266).png" alt=""><figcaption></figcaption></figure>

Here, we will use this d01-prone-f-loop.fbx as a demonstration.



Import this animation FBX in blender

<figure><img src="../.gitbook/assets/image (267).png" alt=""><figcaption></figcaption></figure>



Select the armature, go to the "Animation" panel, and set the window view as "Graph Editor".

<figure><img src="../.gitbook/assets/image (268).png" alt=""><figcaption></figcaption></figure>

In this view, it plots all the keyframe values on the timeline

<figure><img src="../.gitbook/assets/image (269).png" alt=""><figcaption></figcaption></figure>



To repeat the motion, select all the keypoints by pressing `A`, and then press `Shift` + `E`. In the opened menu, select "Make Cyclic (F-Modifier)".&#x20;

<figure><img src="../.gitbook/assets/image (270).png" alt=""><figcaption></figcaption></figure>

For more information on the F-Modifier, please see [here](https://docs.blender.org/manual/en/latest/editors/graph_editor/fcurves/modifiers.html#cycles).



Now the values are repeating, but we also want to offset the global root positions to let the character keep climb forward.

<figure><img src="../.gitbook/assets/image (271).png" alt=""><figcaption></figcaption></figure>



To do this, select a datapoint from the keyframe entry that we want to modify:

<figure><img src="../.gitbook/assets/image (272).png" alt=""><figcaption></figcaption></figure>

Go to the right N-menu of this window, select "Modifiers" panel, and set both the Before Mode and After Mode to be "Repeat with Offset".

<figure><img src="../.gitbook/assets/image (273).png" alt=""><figcaption></figcaption></figure>



Now we can see that the green Y-axis value keeps increasing / decreasing indefinintely.

<figure><img src="../.gitbook/assets/image (274).png" alt=""><figcaption></figcaption></figure>



Final note: this modifier is compatible with our Python API that reads the pose bone values, hence can be used in the PoseLib-V2 reference motion generation pipeline.





## References

{% embed url="https://blender.stackexchange.com/questions/440/how-may-i-create-a-continuously-looping-animation" %}

{% embed url="https://blender.stackexchange.com/questions/132912/repeated-animation" %}
