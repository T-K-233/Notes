# Linux Mounting SD Card



List all the cards by using command. You might want to use sudo priviledge.

```bash
fdisk -l
```

<figure><img src="../../.gitbook/assets/image (15) (2).png" alt=""><figcaption></figcaption></figure>



Create mounting directory by&#x20;

```bash
mkdir /mnt/sdcard
```



Mount

```bash
mount /dev/sdd2 /mnt/sdcard
```



Unmount

```bash
umount /mnt/sdcard
```
