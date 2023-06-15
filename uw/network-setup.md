# Network Setup

Currently using Google Webpass Internet service, which directly offers a in-house ethernet cable. Good thing about this is that we don't need an extra modem to introduce hassle into the system.



The top-level routing is handled by the ASUS RT-AC86U router, with a local LAN IP of `10.0.0.1/16`. The static IP allocation table is shown below.



| Device                          | MAC               | IP         |
| ------------------------------- | ----------------- | ---------- |
| ASUS Router                     | --                | 10.0.0.1   |
| DESKTOP-AVENUE                  | 04:92:26:5E:04:5A | 10.0.0.2   |
|                                 |                   |            |
| LAPTOP-FRAMEWORK                | BC:09:1B:F4:25:EF | 10.0.0.10  |
| DESKTOP-PaperWeight (WiFi)      | D0:C6:37:40:63:93 | 10.0.0.13  |
| ESP32 IP Camera (in white case) | 4C:EB:D6:61:7B:D8 | 10.0.1.1   |
| TKAE ASUS Router                | FC:34:97:CF:B2:00 | 10.0.0.233 |

&#x20;

Then, we have another ASUS RT-N12D1 router on the robot with a local LAN IP of `10.233.0.1/16`. The WAN port of this router is connected to LAN port of the main router.

| Device           | MAC               | IP         |
| ---------------- | ----------------- | ---------- |
| TKAE ASUS Router | --                | 10.233.0.1 |
| TKAE Pi 3        | B8:27:EB:18:46:2A | 10.233.0.2 |



Because accessing devices at `10.233.0.0` from `10.0.0.0` is desired, a LAN static route rule is posted on the main router.

![](<../.gitbook/assets/image (77).png>)



And to access the router from Internet, a port forwarding rule is also posted on the main router, which forwards the main router's 233 port to sub-router 8080 port, which is configured to be the WAN access port.

![](<../.gitbook/assets/image (73).png>)



With this hierarchical design, it is ensured that the system on the robot is standalone, which enables the possibility of connecting to another public network without additional configuration.
