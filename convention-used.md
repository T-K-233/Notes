# Convention Used

This document will obey the following convention:

## File Path

When target is a directory (folder), the path will end with a slash delimiter

e.g.

```bash
cd /home/tk/Downloads/
export PATH=~/Documents/aurora/bin/:$PATH
ls ~/Downloads/
```



When target is a file, the path will end with file extension, if any.

e.g.

```bash
source ./env.sh
wget https://10.0.0.2/motion_file.npy
cat ~/.bashrc
```



When using relative directory in terminal, the path will start with the "current directory" symbol (".")

e.g.

```bash
source ./env.sh
cd ./configs/
ls ./output/
```



## Date and Time

Generic representation

```bash
YYYY-mm-dd HH:MM:ss
```

e.g.

```bash
2023-12-04 00:18:20
```



For Python datetime module, it will be

```
datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
```



When space and other special characters are not allowed in the representation, use

```bash
YYYY-mm-dd_HH-MM-ss
```





