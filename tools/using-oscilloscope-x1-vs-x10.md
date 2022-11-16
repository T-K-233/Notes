# Using Oscilloscope: x1 vs x10

## Rule of thumb

Use **x1** in case the signal level is too low (<1.0V).

Use **x10** for everyday usage.



## How It Works

![](https://www.electronics-notes.com/images/oscilloscope-probe-x10-circuit-01.svg)

x1 will directly connect to oscilloscope using the standard setting

x10 will enable a built-in attenuator in the probe tip, in which can increase the impedance of the signal, thus creating lower load to the circuit-under-test (affect less to the circuit). However, a negative aspect of the x10 settings is, also due to the increased impedance, the amplitude of the signal will be decrease by 10x, and thus cannot be used for low-voltage signals.

## Reference

[What are the differences between a x1 and a x10 oscilloscope probe](https://electronics.stackexchange.com/questions/525712/what-are-the-differences-between-a-x1-and-a-x10-osciloscope-probe)
