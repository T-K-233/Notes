# Solving the .nfsXXX file cannot be deleted issue



First, list the process that is holding the file by

```bash
lsof +D your/path/
```

It will show a bunch of information. The second column is the process ID.

Then, we kill the process by

```bash
kill -9 pid
```







