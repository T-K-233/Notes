# Ethernet - UDP



Set up LWIP according to the previous note







```c

#include "lwip/udp.h"


void UDP_receive_handler(void *arg, struct udp_pcb *udp_control, struct pbuf *packet, const ip_addr_t *addr, u16_t port) {
  struct pbuf *tx_buf;

  // Get the IP of the Client
//  char *remote_ip = ipaddr_ntoa(addr);

  char buf[100];

  int len = sprintf(buf,"Hello %s From UDP SERVER\n", (char*)packet->payload);

  // allocate pbuf from RAM
  tx_buf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

  // copy the data into the buffer
  pbuf_take(tx_buf, buf, len);

  // Connect to the remote client
  udp_connect(udp_control, addr, port);

  // Send a Reply to the Client
  udp_send(udp_control, tx_buf);

  // free the UDP connection, so we can accept new clients
  udp_disconnect(udp_control);

  // Free the buffers
  pbuf_free(tx_buf);
  pbuf_free(packet);
}

void UDP_init_server() {
   /* 1. Create a new UDP control block  */
   struct udp_pcb *udp_control = udp_new();

   /* 2. Bind the upcb to the local port */
   ip_addr_t ip_addr;
   u16_t port = 7000;
   IP_ADDR4(&ip_addr, 10, 0, 64, 64);

   err_t err = udp_bind(udp_control, &ip_addr, port);

   /* 3. Set a receive callback for the upcb */
   if(err == ERR_OK) {
     udp_recv(udp_control, UDP_receive_handler, NULL);
   }
   else {
     udp_remove(udp_control);
   }
}
```



and then in the main function,&#x20;

```c
  /* USER CODE BEGIN 2 */

  UDP_init_server();
  
  /* USER CODE END 2 */
```





We can use a Python script to test it

```python
import socket


ADDR = "10.0.64.64"
PORT = 7000

# open UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# send data
sock.sendto(b"Hello, world!", (ADDR, PORT))

print(f"Sent: Hello, world! to {ADDR}:{PORT}")

# receive data
data, addr = sock.recvfrom(1024)

print(f"Received: {data.decode()} from {addr}")

```





