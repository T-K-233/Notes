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





## Synth

```tcl
reset_run synth_1

launch_runs synth_1 -jobs 8
```





## Bitstream

```
launch_runs impl_1 -to_step write_bitstream -jobs 8
```





## Generate IP

{% code overflow="wrap" %}
```tcl
create_ip -name clk_wiz -vendor xilinx.com -library ip -version 6.0 -module_name clk_wiz_0

set_property -dict [list \
  CONFIG.CLKOUT1_JITTER {193.154} \
  CONFIG.CLKOUT1_PHASE_ERROR {109.126} \
  CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {20} \
  CONFIG.MMCM_CLKFBOUT_MULT_F {8.500} \
  CONFIG.MMCM_CLKOUT0_DIVIDE_F {42.500} \
] [get_ips clk_wiz_0]

generate_target {instantiation_template} [get_files /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci]

generate_target all [get_files  /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci]

catch { config_ip_cache -export [get_ips -all clk_wiz_0] }

export_ip_user_files -of_objects [get_files /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci] -no_script -sync -force -quiet
create_ip_run [get_files -of_objects [get_fileset sources_1] /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci]
launch_runs clk_wiz_0_synth_1 -jobs 8

export_simulation -of_objects [get_files /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci] -directory /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.ip_user_files/sim_scripts -ip_user_files_dir /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.ip_user_files -ipstatic_source_dir /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.ip_user_files/ipstatic -lib_map_path [list {modelsim=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/modelsim} {questa=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/questa} {xcelium=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/xcelium} {vcs=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/vcs} {riviera=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/riviera}] -use_ip_compiled_libs -force -quiet

```
{% endcode %}



GPIO

{% code overflow="wrap" %}
```tcl
create_ip -name axi_gpio -vendor xilinx.com -library ip -version 2.0 -module_name axi_gpio_0

generate_target {instantiation_template} [get_files /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/ip/axi_gpio_0/axi_gpio_0.xci]

update_compile_order -fileset sources_1
generate_target all [get_files  /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/ip/axi_gpio_0/axi_gpio_0.xci]

catch { config_ip_cache -export [get_ips -all axi_gpio_0] }

export_ip_user_files -of_objects [get_files /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/ip/axi_gpio_0/axi_gpio_0.xci] -no_script -sync -force -quiet

export_simulation -of_objects [get_files /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.srcs/sources_1/ip/axi_gpio_0/axi_gpio_0.xci] -directory /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.ip_user_files/sim_scripts -ip_user_files_dir /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.ip_user_files -ipstatic_source_dir /home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.ip_user_files/ipstatic -lib_map_path [list {modelsim=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/modelsim} {questa=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/questa} {xcelium=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/xcelium} {vcs=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/vcs} {riviera=/home/tk/Downloads/MinimalArty/Arty35T-Playground/Arty35T-Playground.cache/compile_simlib/riviera}] -use_ip_compiled_libs -force -quiet
```
{% endcode %}





