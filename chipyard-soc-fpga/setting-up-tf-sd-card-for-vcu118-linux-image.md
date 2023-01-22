# Preparing SD Card for vcu118 Linux Image



List all disks

```bash
sudo fdisk -l
```



remove all existing partitions

```bash
sudo gdisk /dev/sda

Command (? for help): d
Partition number (1-2): 1

Command (? for help): d
Using 2

Command (? for help): w

Final checks complete. About to write GPT data. THIS WILL OVERWRITE EXISTING PARTITIONS!!

Do you want to proceed? (Y/N): Y
```



create new partition

we need to enable expert mode to be able to create partition strictly at sector 34

```bash
sudo gdisk /dev/sda
x
l
1
m

Command (? for help): n
Partition number (1-128, default 1): 1
First sector (34-11721045134, default = 2048) or {+-}size{KMGTP}: 34
Last sector (2048-11721045134, default = 11721045134) or {+-}size{KMGTP}: 65536
Current type is 8300 (Linux filesystem)
Hex code or GUID (L to show codes, Enter = 8300): 700

Command (? for help): w

Final checks complete. About to write GPT data. THIS WILL OVERWRITE EXISTING PARTITIONS!!

Do you want to proceed? (Y/N): Y
OK; writing new GUID partition table (GPT) to /dev/sda.
The operation has completed successfully.

```



```bash
sudo dd if=~/Desktop/eaglex_bringup/eaglex_bbl.bin of=/dev/sdb1

32768+0 records in
32768+0 records out
16777216 bytes (17 MB, 16 MiB) copied, 3.21026 s, 5.2 MB/s

```









