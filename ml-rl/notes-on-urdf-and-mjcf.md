# Notes on URDF and MJCF

It is important to note the slight differences between the URDF and MJCF formats to avoid bugs. In this writeup I will try to list down some common pitfalls on working with these two formats.



### Box size

URDF defines a box size with the three side lengths, while MJCF specifies **half-size** length.

<figure><img src="../.gitbook/assets/image (278).png" alt=""><figcaption></figcaption></figure>

### Isaac Gym: Segmentation Fault When Adding a URDF Actor

```bash
Adding actor 'humanoid-urdf/gym/urdf/robot.urdf' to env 0
Segmentation fault (core dumped)
```

This error could due to multiple reasons, and here's an incomplete checklist

* [ ] Make sure there are \<collision> tags in the link
* [ ] Make sure mesh filename are correct
* [ ] Make sure there is at least one DoF
