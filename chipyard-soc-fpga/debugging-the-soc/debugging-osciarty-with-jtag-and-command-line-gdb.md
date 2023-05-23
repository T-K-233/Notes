# Debugging OsciArty with JTAG and command line GDB

Launching gdb

```bash
elf="build/firmware.elf"
gdb -ex "target extended-remote localhost:3333" \
    -ex "monitor reset init" \
    -ex "load ${elf}" \
    -ex "continue" \
    "${elf}"
```





```
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug",
            "type": "gdb",
            "cwd": "${workspaceRoot}",
            "request": "attach",
            "executable": "${workspaceRoot}/firmware/build/firmware.elf",
            "target": "localhost:3333",
            
            "valuesFormatting": "parseText",
            "gdbpath": "riscv64-unknown-elf-gdb",
            "remote": true,
            "autorun": [
                "load firmware/build/firmware.elf",
                // "break main",
                "monitor reset"
            ]
        }
    ]
}
```



[https://www.justinmklam.com/posts/2017/10/vscode-debugger-setup/](https://www.justinmklam.com/posts/2017/10/vscode-debugger-setup/)







pipe connection:

[https://openocd.org/doc/html/GDB-and-OpenOCD.html](https://openocd.org/doc/html/GDB-and-OpenOCD.html)

"""

A pipe connection is typically started as follows:

```
target extended-remote | \
       openocd -c "gdb_port pipe; log_output openocd.log"
```





### Fix VSCode cannot open COM port issue on Linux machines

```bash
sudo chmod a+rw /dev/ttyUSB3
```









