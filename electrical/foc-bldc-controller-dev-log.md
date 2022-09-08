# FOC BLDC Controller Dev Log

配置 ADC1 和 ADC2 分别读取 Opamp 1 和 Opamp 2 之后的结果 (Polling mode)

可以看出来相位差 1/3，说明确实读取成功了

![](<../.gitbook/assets/image (140).png>)

接下来将 ADC 配置成 DMA 模式

配置的过程颇为艰辛，还发现了原作者的一个 BUG

首先将 ADC Trigger Conversion Mode 设置成 TIM1 的 TRGO，并且在 TIM1 那里将 TRGO source 设置为 Counter Update。在 center align mode 下面，每次 counter 到顶/底都会进行一次 update。

然后配置 DMA 设置，这里 ADC1 先使用 1 channel (rank 1)，ADC2 使用 2 channels (rank 1 & rank 2)。

DMA 设置成 Circular，并且将 DMA Continuous Requests 设置为 Enable，这样应该就能自动让 ADC 在 TIM1 Trigger 下自动 conversion，initiate DMA Transfer，并在下个周期重复执行。

原作者是在 ADC Callback 里面手动再 ADC\_Start\_DMA。这样的问题是第一个 rank 读不到数据，所以作者把两个 ADC 的 rank 1 都在 rank 2 中重复了一遍。

下面是配置完完整三个 OpAmp 之后读出来的电机波形&#x20;

![](<../.gitbook/assets/image (82).png>)

![](<../.gitbook/assets/image (11).png>)

![](<../.gitbook/assets/image (117).png>)

![](<../.gitbook/assets/image (31).png>)
