# Remove EFI disk partition

Launch cmd, execute

```bash
diskpart
```



In diskpart, do

```bash
DISKPART> list disk
```



Select target disk, where N is the target disk index

```bash
DISKPART> select disk N
```



```bash
DISKPART> list partition
```

```bash
DISKPART> select partition
```



```bash
DISKPART> clean
DISKPART> create partition primary
```



Then, delete this new primary partition in Disk Management window, and create new partition as desired.
