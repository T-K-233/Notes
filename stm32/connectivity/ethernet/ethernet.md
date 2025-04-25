# Ethernet - LWIP

## Setting up Ethernet PHY Layer

Under "Connectivity" tab, select ETH.

Set ETH mode to RMII, and make sure the pin mapping corresponds to the one on the Nucleo board

<figure><img src="../../../.gitbook/assets/image (2) (1) (1) (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

{% tabs %}
{% tab title="NUCLEO-F429ZI" %}
<figure><img src="../../../.gitbook/assets/image (1) (3) (1).png" alt=""><figcaption></figcaption></figure>

On the F429ZI Nucleo board, the pin mapping should look as follows.

<figure><img src="../../../.gitbook/assets/image (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>
{% endtab %}

{% tab title="NUCLEO-H755ZI" %}
<figure><img src="../../../.gitbook/assets/image (219).png" alt=""><figcaption></figcaption></figure>

On the H755ZI Nucleo board, the pin mapping should look as follows.

<figure><img src="../../../.gitbook/assets/image (220).png" alt=""><figcaption></figcaption></figure>
{% endtab %}
{% endtabs %}





## Setting up Middleware LWIP Protocol Layer

Under "Middleware and Software Packs" tab, select LWIP (Lightweight IP).

Enable the Lightweight IP functionality.

The Platform Settings will mark a warning icon. We will configure this at the last step.

<figure><img src="../../../.gitbook/assets/image (2) (1) (1) (1) (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>



Disable DHCP and manually set the IP

<figure><img src="../../../.gitbook/assets/image (1) (2) (1) (1).png" alt=""><figcaption></figcaption></figure>

set MEM\_SIZE to 10K

<figure><img src="../../../.gitbook/assets/image (7) (2).png" alt=""><figcaption></figcaption></figure>

Now, we configure the Platform Settings. For Nucleo 144 boards (F429, H755 etc.), we need to set to LAN8742, which corresponds to the U9 PHY IC on the board.

<figure><img src="../../../.gitbook/assets/image (6) (2) (1).png" alt=""><figcaption></figcaption></figure>





```c
/* USER CODE BEGIN 0 */

extern struct netif gnetif;

/* USER CODE END 0 */

...

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
    ethernetif_input(&gnetif);
  }
  /* USER CODE END 3 */

```





## Testing

After configuration, the STM32 should be able to respond to ping requests.





## Reference

{% embed url="https://controllerstech.com/stm32-ethernet-tutorials/" %}

