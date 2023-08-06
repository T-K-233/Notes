# SAI Color Flip/Color Inversion

SAI does not provide color flip/color inversion functionality directly, but we can achieve this flip with blend mode.



For example, if we want to flip the following image:

<figure><img src="../.gitbook/assets/mumei_style.png" alt=""><figcaption></figcaption></figure>

Create another layer **above** our target image, and set the blend mode as **"Exclude"**.

<figure><img src="../.gitbook/assets/image (157).png" alt=""><figcaption></figcaption></figure>

Now we get the inverted image.

<figure><img src="../.gitbook/assets/mumei_style_black.png" alt=""><figcaption></figcaption></figure>

