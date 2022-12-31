# MIT Motor Controller Code Analyze

{% embed url="https://github.com/bgkatz/motorcontrol" %}

## Clock Tree

<figure><img src="../.gitbook/assets/image (2).png" alt=""><figcaption></figcaption></figure>

## NVIC Settings

<figure><img src="../.gitbook/assets/image (3).png" alt=""><figcaption></figcaption></figure>





## PowerStage PWM Generation

<figure><img src="../.gitbook/assets/image.png" alt=""><figcaption></figcaption></figure>

Using DIV/1 prescaler, 0x8CA (2250) period, and center aligned mode -> PWM frequency is 40kHz

With the repetition counter set to 1, the software interrupt frequency is 20kHz.



## Encoder

<figure><img src="../.gitbook/assets/image (1).png" alt=""><figcaption></figcaption></figure>

It's using blocking encoder read/write.



