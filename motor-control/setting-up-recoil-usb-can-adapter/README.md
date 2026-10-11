# Setting up Recoil USB-CAN Adapter

## Ubuntu

The device driver for the adapter is already included in the linux driver libraries.&#x20;

The device should appear on the `/dev/ttyACM0` port.&#x20;

### Change port access

By default, the port is only accessible to sudo users. Run the following command to create or edit the config file

```bash
sudoedit /etc/udev/rules.d/50-ttyacm.rules
```

In the opened editor, add the following lines:

```bash
KERNEL=="ttyACM[0-9]*",MODE="0666"
```

After saving, unplug and re-plug the device. Now it should be accessible to all users.

For more on udev rules, see [Configure USB Access Permissions (udev rules) on Ubuntu](../../tools/ubuntu/configure-usb-access-permissions-on-ubuntu.md).

## Windows

### 1. Plug in the device

### 2. Update device driver

Open Zadig

Under "Options", select "List All Devices"

<figure><img src="../../.gitbook/assets/image (9) (2).png" alt=""><figcaption></figcaption></figure>

In the dropdown menu, select "Recoil USB-CAN Adapter"

<figure><img src="../../.gitbook/assets/image (11) (1).png" alt=""><figcaption></figcaption></figure>

Set the target driver to be "USB Serial (CDC)", and click "Replace Driver"

<figure><img src="../../.gitbook/assets/image (1) (1) (4).png" alt=""><figcaption></figcaption></figure>

### 3. Verify Install

After driver installation, the device should appear in the Device Manager.

<figure><img src="../../.gitbook/assets/image (1) (1) (2).png" alt=""><figcaption></figcaption></figure>
