# Check Disk / Folder / File Size



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



