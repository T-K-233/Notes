# Conventions in this doc



## Path

When target is a directory (folder), the path will end with a slash delimiter

#### e.g.

```bash
cd /home/tk/Downloads/
export PATH=~/Documents/aurora/bin/:$PATH
ls ~/Downloads/
```



When target is a file, the path will end with file extension, if any.

#### e.g.

```bash
source ./env.sh
wget https://10.0.0.2/motion_file.npy
cat ~/.bashrc
```



When using relative directory in terminal, the path will start with the "current directory" symbol (".")

#### e.g.

```bash
source ./env.sh
cd ./configs/
ls ./output/
```



