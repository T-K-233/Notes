# A Study on Reset Pin Connection

We will use the STM32F446 Nucleo board to conduct the study.

{% embed url="https://www.st.com/en/evaluation-tools/nucleo-f446re.html" %}





From the schematic, we can observe that both the STM32F446 chip and the STM32F103 chip (used as the ST-LINK debugger) uses a 100nF capacitor between NRST pin and GND. Additionally, the STM32F103 uses an external 100k pull-up resistor.

<figure><img src="../.gitbook/assets/image (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (2) (1) (1) (1) (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

This matches the recommended reset circuit in the datasheet. $$R_{PU}$$ is typically 40k.

<figure><img src="../.gitbook/assets/image (3) (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>



Here is a simplified circuit diagram of the reset pin circuit. On Nucleo board the optional 100R resistor is not used.

<figure><img src="https://lh7-us.googleusercontent.com/vjJWvKQsrNxyiIXFzl7YSI9SU6m-h_1wyQaOOwzqinBQkB-XudRS4xO-N7zFEiSBDtlZ8vfUKdK_k_85oc7pa_RrNwY9gwofAmsQGp2zTK-E0QuGMKS2XpVfykZSQiYvYBzfVVrugD55q7eT8of7GsZ_Sw=s2048" alt=""><figcaption></figcaption></figure>





Now we probe the reset pin voltage level. When the reset button is pressed, the reset pin drops to 0V immediately, and after the button is released, the voltage gradually raises back.

The input Schmit trigger has a high level of 0.7 \* VDD, which when VDD is 3.3V, the trigger will detect signal voltage > 2.3 V as a valid high signal. The reset signal takes 5.3 ms after the button release to release the chip from reset state.

<figure><img src="https://lh7-us.googleusercontent.com/fryiTRJptSJVPIY2nmpI6_H1h6WfQ1kwaZMhPZM4ipUNdjG4LRLkN6ThFT-VTJdd0l1OusSPCFU9iLFnIdonlgq5rkhLrg77G2iHoK21XX-2Zh45x0GxYydnGK-wjhuGRTO2DZY2qXC3IgC1ZlgxzYuC1w=s2048" alt=""><figcaption></figcaption></figure>



The capacitor also functions as a power-on-reset. Here, yellow channel probes the reset voltage, and magenta channel probes the 3V3 voltage. The MCU is released from reset after 59.4 ms after the board is powered on (most of the time is controlled by the ST-LINK debugger), which gives sufficient time for components on the chip to properly initialize.

<figure><img src="../.gitbook/assets/image (5) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>



