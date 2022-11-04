# Ethernet



Set ETH mode to RMII, and make sure the pin mapping correspond to the one on the Nucleo board



<figure><img src="../../.gitbook/assets/image (4) (3).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (1) (3).png" alt=""><figcaption></figcaption></figure>



Enable Lightweight IP

Disable DHCP and manually set the IP&#x20;

<figure><img src="../../.gitbook/assets/image (1) (2) (1).png" alt=""><figcaption></figcaption></figure>

set MEM\_SIZE to 10K

<figure><img src="../../.gitbook/assets/image (7).png" alt=""><figcaption></figcaption></figure>



For Nucleo 144 board, we need to set to LAN8742

<figure><img src="../../.gitbook/assets/image (6).png" alt=""><figcaption></figcaption></figure>



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









