# Unreal Engine Communicate with SteamVR and External Python

## 0. Environment

Windows 10

Unreal Engine 5.1.1

SteamVR 1.25.8

## 1. Create project

<figure><img src="../.gitbook/assets/image (13) (2).png" alt=""><figcaption></figcaption></figure>

## 2. Add plugin

<figure><img src="../.gitbook/assets/image (14) (1).png" alt=""><figcaption></figcaption></figure>

Add the following plugins

<figure><img src="../.gitbook/assets/image (9) (2) (2) (1).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (8) (2).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (1) (7).png" alt=""><figcaption></figcaption></figure>

Unreal Engine needs a restart.

Before restart, make sure SteamVR is running

<figure><img src="../.gitbook/assets/image (10) (2).png" alt=""><figcaption></figcaption></figure>

## 3. Set up VR devices

<figure><img src="../.gitbook/assets/image (3) (1).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (2) (1) (1).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/image (18) (1).png" alt=""><figcaption></figcaption></figure>

## 4. Bind to virtual camera

Create a virtual camera.

<figure><img src="../.gitbook/assets/image (7).png" alt=""><figcaption></figcaption></figure>

Add a "Live Link Controller" component to VCam.

<figure><img src="../.gitbook/assets/image (2) (1) (1) (4).png" alt=""><figcaption></figcaption></figure>

In the "Subject Representation" field, select the desired VR device to bind.

<figure><img src="../.gitbook/assets/image (11) (2).png" alt=""><figcaption></figcaption></figure>

Now the virtual camera should be controlled by the VR device.





## Reference

{% embed url="https://www.youtube.com/watch?v=HwAra3yXVHs" %}

{% embed url="https://www.youtube.com/watch?v=pqbuk0nQ5Sk" %}

Other possible methods:

{% embed url="https://dev.epicgames.com/community/learning/tutorials/LdR2/unreal-engine-how-to-use-vive-trackers-in-ue-5-2-and-later-versions" %}

{% embed url="https://github.com/ValveSoftware/openxr_engine_plugins" %}





{% embed url="https://docs.unrealengine.com/4.26/en-US/BlueprintAPI/SteamVR/GetTrackedDevicePositionandOrien-/" %}
