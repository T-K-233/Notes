# Notes on URDF and MJCF

It is important to note the slight differences between the URDF and MJCF formats to avoid bugs. In this writeup I will try to list down some common pitfalls on working with these two formats.



### Box size

URDF defines a box size with the three side lengths, while MJCF specifies **half-size** length.

<figure><img src="../.gitbook/assets/image (278).png" alt=""><figcaption></figcaption></figure>













