# Test Disk Read/Write Speed

## 1. Create a large file

Here, we create a tmp.txt file with size of 2 Gigabytes

```bash
truncate -s 2G ./tmp.txt
```



## 2. File Transfer

```bash
dd if=./tmp.txt of=./tmp.out bs=1G count=1024
```

