# Unreal Engine Socket Communication



## 0. Environment

Windows 10

Unreal Engine 5.1.1

SteamVR 1.25.8



## 1. Add plugin

<figure><img src="../.gitbook/assets/image (19).png" alt=""><figcaption></figcaption></figure>



## 2. Create a new GameModeBase class

<figure><img src="../.gitbook/assets/image (8).png" alt=""><figcaption></figcaption></figure>



Choose "GameModeBase" as the parent class.

<figure><img src="../.gitbook/assets/image (9).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (10).png" alt=""><figcaption></figcaption></figure>



<figure><img src="../.gitbook/assets/image (5).png" alt=""><figcaption></figcaption></figure>



Finally, update the GameModeBase class

<figure><img src="../.gitbook/assets/image (11).png" alt=""><figcaption></figcaption></figure>

## 3. External Python server

```python
import socket
import struct

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("", 8080))
s.listen(1)

conn, addr = s.accept()

print("connected by", addr)

while True:
    data = conn.recv(1024)

    print("recv:", data)

    content = "hello Unreal!"

    buffer = struct.pack(">BB", 1, len(content))
    buffer += content.encode()

    conn.sendall(buffer)
    print("sent:", buffer)

    break

s.close()

```

