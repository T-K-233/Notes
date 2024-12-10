---
description: Using Timer on STM32
---

# Using TIM on STM32







## Timer TRGO Events

<figure><img src="../../.gitbook/assets/image (248).png" alt=""><figcaption></figcaption></figure>

#### Reset

This is rarely used. Software control of the slave device is preferred.

#### Enable

This is rarely used. Software control of the slave device is preferred.

#### Update Event

This event is generated when counter overflows or underflows.

#### Capture Compare

This event is generated when the counter is equal to the channel pulse value. Think this as the PWM signal.&#x20;

However, note that when using "Active level on match" or "Inactive level on match", it will only generate one trigger event on TRGO and will not be reset.



## Repetition counter settings

<figure><img src="../../.gitbook/assets/image (249).png" alt=""><figcaption></figcaption></figure>







## PWM generation mode

### To Drive Active Low LEDs

set CH Polarity to Low.

<figure><img src="../../.gitbook/assets/image (3) (5).png" alt=""><figcaption></figcaption></figure>

Do not change PWM Mode. The internal logic is still operating in "PWM mode 1" mode.

> e.g. Setting CCR to 0 makes the channel always inactive, while setting CCR to ARR makes the channel always active.

<figure><img src="../../.gitbook/assets/image (2) (6) (1).png" alt=""><figcaption></figcaption></figure>
