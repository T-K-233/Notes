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
source /tools/flexlm/flexlm.s
```



```bash
bsub -q ee194 -Is -XF make CONFIG=BearlyConfig
```







