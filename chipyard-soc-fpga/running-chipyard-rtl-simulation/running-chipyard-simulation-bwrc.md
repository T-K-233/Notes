# Running Chipyard Simulation - BWRC

## 1. Prepare Environment

On BWRC machine, we need to source the vcs simulator path. This can be done by executing the following commands.

```bash
export VCS_HOME=/tools/synopsys/vcs/S-2021.09-SP1-1/
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

if run into JDK\_HOME issue (JDK/lib/tools.jar not found), do this:

```bash
export JDK_HOME=/usr/lib/jvm/java-1.8.0/
```

## 2. Run Simulation

```bash
cd sims/vcs/
```

To build simulation code:

```bash
bsub -q ee194 -Is -XF make CONFIG=BearlyConfig
```

To run the simulation with a program:

<pre class="language-bash"><code class="lang-bash"><strong>bsub -q ee194 -Is -XF make run-binary CONFIG=BearlyConfig BINARY=../../tests/hello.riscv
</strong></code></pre>

To run the simulation with a program, and generate waveform:

timeout\_cycles can be set to a small value to make the waveform dump process faster

```bash
bsub -q ee194 -Is -XF make run-binary-debug CONFIG=BearlyConfig BINARY=../../tests/hello.riscv timeout_cycles=10000
```

## 3. Examine Waveform

Launch verdi to examine the waveform

```bash
bsub -q ee194 -Is -XF verdi
```
