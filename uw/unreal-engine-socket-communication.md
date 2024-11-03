# Unreal Engine Socket Communication

## 0. Environment

Windows 10

Unreal Engine 5.1.1

SteamVR 1.25.8

## 1. Create a new Unreal project

Select "Film / Video & Live Events" -> "Blank". Leave "Starter Content" and "Raytracing" unchecked.

Click "Create" button.

<figure><img src="../.gitbook/assets/image (13) (2).png" alt=""><figcaption></figcaption></figure>

## 2. Add plugin

Click "Edit" -> "Plugins" to open the plugins window.

<figure><img src="../.gitbook/assets/image (14) (1).png" alt=""><figcaption></figcaption></figure>

Search and add the following plugin.

<figure><img src="../.gitbook/assets/image (19) (3) (1).png" alt=""><figcaption></figcaption></figure>

After adding the plugin, Unreal Engine needs to be restarted.



## 3. Set up a new GameModeBase class

Click the Blueprint button, select "New Empty Blueprint Class...".

<figure><img src="../.gitbook/assets/image (8) (1) (2).png" alt=""><figcaption></figcaption></figure>

Choose "GameModeBase" as the parent class.

<figure><img src="../.gitbook/assets/image (9) (2) (2) (2).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (10) (1).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (5) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

Finally, update the GameModeBase class

<figure><img src="../.gitbook/assets/image (11) (1) (2) (1).png" alt=""><figcaption></figcaption></figure>

## 4. External Python server

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





### A Note on FPS



Since we might want to use EventTick to transmit messages, it could be useful if we limit event tick rate on the gamemode base.



To set the rate, go to Blueprint setting of the GameModeBase, select "GameModeBase" on the Components view. Then, on the Details panel, change the "Tick Interval" value to desired value.

<figure><img src="../.gitbook/assets/image (190).png" alt=""><figcaption></figcaption></figure>

