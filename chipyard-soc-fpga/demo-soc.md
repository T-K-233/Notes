# Demo SoC

```bash
export VCS_HOME=/tools/synopsys/vcs/T-2022.06-SP2/
export VERDI_HOME=/tools/synopsys/verdi/S-2021.09-SP1-1/
export VCS_64=1
export PATH=$VCS_HOME/bin:$VERDI_HOME/bin:$PATH

```





bsub -q ee194 -Is -XF make CONFIG=BearlyConfig



