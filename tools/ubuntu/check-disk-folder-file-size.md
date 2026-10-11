# Disk and SD Card Commands

## Check disk size

```bash
df -h .
```

```bash
(base) tk@a25:/rscratch/tk/Desktop$ df -h .
Filesystem                        Size  Used Avail Use% Mounted on
10.11.49.77:/export/zfs/rscratch  115T  101T   15T  88% /rscratch
```

## Check Current Folder Size

```bash
du -sh .
```

```bash
(base) tk@a25:/rscratch/tk/Desktop$ du -sh .
88G     .
```

To list children sizes, do

```bash
du -ch .
```

## Test Disk Read/Write Speed

### 1. Create a large file

Here, we create a tmp.txt file with size of 2 Gigabytes

```bash
truncate -s 2G ./tmp.txt
```

### 2. File Transfer

```bash
dd if=./tmp.txt of=./tmp.out bs=1G count=1024
```

## Mounting SD Card

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

## Partitioning SD Card

### Errata

If encounter the following error showing that the disk is read-only, make sure that the lock pin on the SD card is switched to near the pin-side of the card, such that the card is unlocked and writable.

<figure><img src="../../.gitbook/assets/image (187).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (188).png" alt=""><figcaption><p>In "Disks" it also shows the disk is "Read-Only"</p></figcaption></figure>
