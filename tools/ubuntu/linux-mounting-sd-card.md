# Linux Mounting SD Card

List all the cards by using command. You might want to use sudo priviledge.

```bash
fdisk -l
```

<figure><img src="../../.gitbook/assets/image (15) (2).png" alt=""><figcaption></figcaption></figure>

Create mounting directory by

```bash
mkdir /mnt/sdcard
```

Mount

```bash
sudo mount /dev/sdd2 /mnt/sdcard
```

after mounting, reload the disk with the following command

```bash
systemctl daemon-reload
```



Unmount

```bash
sudo umount /mnt/sdcard
```
