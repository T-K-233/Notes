# Running Chipyard Simulation

On BWRC machine, we need to source the vcs simulator path. This can be done by executing the following commands.

```bash
export VCS_HOME=/tools/synopsys/vcs/T-2022.06-SP2/
export VERDI_HOME=/tools/synopsys/verdi/S-2021.09-SP1-1/
export VCS_64=1
export PATH=$VCS_HOME/bin:$VERDI_HOME/bin:$PATH
```

Or, run this:

```bash
source /tools/C/chiyufeng/documents/vcs_env.sh
```



Might also need to run this command to get license

```bash
source /tools/flexlm/flexlm.sh
```



if run into JDK\_HOME issue (tools.jar not found), do this:

```bash
export JDK_HOME=/usr/lib/jvm/java-1.8.0/
```



```bash
bsub -q ee194 -Is -XF make CONFIG=BearlyConfig
```

<pre class="language-bash"><code class="lang-bash"><strong>make run-binary-debug CONFIG=BearlyConfig BINARY=../../tests/hello.riscv
</strong></code></pre>



```bash
bsub -q ee194 -Is -XF make run-binary-debug CONFIG=BearlyConfig BINARY=../../tests/hello.riscv timeout_cycles=10000

```



```bash
bsub -q ee194 -Is -XF verdi
```



```systemverilog
_core_io_imem_req_bits_pc
```









