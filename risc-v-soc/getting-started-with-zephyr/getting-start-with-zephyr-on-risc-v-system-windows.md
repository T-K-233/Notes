# Getting Start with Zephyr on RISC-V System - Windows





```powershell
python -m venv .\.venv\
```

```powershell
.\.venv\Scripts\Activate.ps1
```

```powershell
pip install west
```



Create project

```powershell
west init zephyrproject
cd .\zephyrproject\
west update
```

When doing this, make sure VSCode is not opened. [https://github.com/zephyrproject-rtos/west/issues/558#issuecomment-1006077681](https://github.com/zephyrproject-rtos/west/issues/558#issuecomment-1006077681)



```powershell
west zephyr-export
```





