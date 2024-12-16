# Vivado TCL Scripts

## Create Project

{% code overflow="wrap" %}
```tcl
create_project Arty35T-Playground /home/tk/Downloads/MinimalArty/Arty35T-Playground -part xc7a35ticsg324-1L
```
{% endcode %}



```tcl
set_property board_part digilentinc.com:arty-a7-35:part0:1.1 [current_project]
```



## Add Files

{% code overflow="wrap" %}
```tcl
update_compile_order -fileset sources_1
file mkdir /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/new
close [ open /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/new/Arty35TShell.v w ]

add_files /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/new/Arty35TShell.v

update_compile_order -fileset sources_1
```
{% endcode %}



{% code overflow="wrap" %}
```bash
file mkdir /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/constrs_1/new
close [ open /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/constrs_1/new/Arty-A7-35-Master.xdc w ]
add_files -fileset constrs_1 /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/constrs_1/new/Arty-A7-35-Master.xdc
```
{% endcode %}



## Bitstream

```
launch_runs impl_1 -to_step write_bitstream -jobs 8
```







