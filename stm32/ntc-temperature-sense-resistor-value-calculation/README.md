# NTC Temperature Sense Resistor Value Calculation



NTC resistance follows the formula

<figure><img src="../../.gitbook/assets/image (2).png" alt=""><figcaption></figcaption></figure>

Take the 10k [NCP18XH103F03RB](https://www.murata.com/en-eu/api/pdfdownloadapi?cate=\&partno=NCP18XH103F03RB) as example:

`T_R` is 25℃, or 298.15 K

`R_R` is 10k at 25℃

`B` is 3380K

Thus, we get

```c
r_ntc = 1e5 * pow(3380. * (1. / temperature - 1. / 298.15));
```

or, to get temperature,

```c
temperature = 3380. / (log(r_ntc / 10000.) + 3380. / 298.15);
```



### Reference Manual

{% embed url="https://www.tdk-electronics.tdk.com/download/531116/19643b7ea798d7c4670141a88cd993f9/pdf-general-technical-information.pdf" %}
