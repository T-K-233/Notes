# TIM







### To Drive Active Low LEDs

set CH Polarity to Low.

<figure><img src="../../.gitbook/assets/image.png" alt=""><figcaption></figcaption></figure>

Do not change PWM Mode. The internal logic is still operating in "PWM mode 1" mode.

> e.g. Setting CCR to 0 makes the channel always inactive, while setting CCR to ARR makes the channel always active.

<figure><img src="../../.gitbook/assets/image (2).png" alt=""><figcaption></figcaption></figure>
