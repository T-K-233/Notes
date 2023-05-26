# Current Loop PI Value Tunin

## PI Controllers

There are two configurations of PI controller, the parallel configuration (most common one) and the series configuration.

### Parallel PI Controller

For parallel configuration, we have

$$
c(t) = k_p e(t) + k_i \int e(t) dt
$$

$$
C(s) = k_p + \frac{k_i}{s} = \frac{k_p s + k_i}{s}
$$

<figure><img src="../.gitbook/assets/image (19).png" alt=""><figcaption></figcaption></figure>



### Serial PI Controller

For series configuration, we have

$$
c(t) = k_p' e(t) + k_p' k_i' \int e(t) dt
$$

$$
C(s) = k_p' (1 + \frac{k_i}{s}) = \frac{k_p's + k_p'k_i'}{s}
$$

<figure><img src="../.gitbook/assets/image (2).png" alt=""><figcaption></figcaption></figure>

The C implementation is

```c
float kp, ki;
float target, measured;
float limit;  // integrator anti-windup value
float dt;     // loop execution time
float integrator;

float error = target - measured;
integrator = clampf(integrator + kp * ki * error * dt, -limit, limit);
float result = kp * error + integrator;

```



## Model of Motor

The motor can be seen as a resistor and inductor in series.

$$
V = IR + L\frac{dI}{dt} + k_{emf}\omega
$$

Since we are focusing on low-rpm scenario, the emf term can be ignored.

$$
V = IR + L\frac{dI}{dt}
$$

performing Laplace transform we get

$$
V(s) = I(s)R + LsI(s) - LI(0)
$$

assuming initial condition I(0) = 0,

$$
V(s) = I(s)R + LsI(s)
$$

$$
M(s) = \frac{I(s)}{V(s)} = \frac{1}{R + Ls} = \frac{\frac{1}{R}}{1 + \frac{L}{R}s}
$$

## System Model

For open loop control, we have the transfer function as

$$
G_{open}(s) = M(s)C(s) = \frac{\frac{1}{R}}{1 + \frac{L}{R}s} \frac{k_p's + k_p'k_i'}{s}
$$

$$
G_{open}(s) = \frac{\frac{1}{R}}{1 + \frac{L}{R}s} k_p'k_i' \frac{\frac{s}{k_i'} + 1}{s}
$$

In order to simplify the equation, we need

$$
1 + \frac{L}{R}s = \frac{s}{k_i'} + 1
$$

$$
k_i' = \frac{R}{L}
$$

Then the equation can be simplified as

$$
G_{open}(s) = k_p'k_i' \frac{\frac{1}{R}}{s}
$$



For closed loop feedback control, we have the transfer function as

$$
G_{closed}(s) = \frac{G_{open}(s)}{1 + G_{open}(s)}
$$

Therefore

$$
G_{closed} = \frac{k_p'k_i' \frac{\frac{1}{R}}{s}}{1 + k_p'k_i' \frac{\frac{1}{R}}{s}}
$$

substitute k\_i' = R/L into the equation

$$
G_{closed} = \frac{k_p'\frac{R}{L} \frac{\frac{1}{R}}{s}}{1 + k_p'\frac{R}{L} \frac{\frac{1}{R}}{s}} = \frac{k_p' \frac{1}{L}}{s + k_p '\frac{1}{L}} = \frac{1}{\frac{s}{\frac{k_p'}{L}} + 1}
$$

The coefficient k\_p' / L determines the frequency where the system response decreases by 3dB, or, the response cutoff bandwidth ω. The unit is rad/s.

In other words, we have

$$
k_p' = L * \omega_{cutoff}
$$

$$
\omega_{cutoff} = \frac{k_p'}{L}
$$

with ω = 2πf,

$$
f_{cutoff} = \frac{1}{2\pi}\frac{k_p'}{L}
$$

Usually in practice, we want to ensure that the current sampling rate is 10% of the switching frequency, i.e. 20% of the Nyquist rate.

$$
k_{p, max}' = \frac{2\pi}{10} T_{sample} L = \frac{2\pi}{10} f_{sample} L
$$



For example, if the switching frequency is 20kHz, the maximum bandwidth would be 2kHz.

$$
k_{p, 2kHz}' = 2 \pi L * 2*10^3
$$

## Reference

Digital PI Controller Equations [https://e2e.ti.com/cfs-file/\_\_key/communityserver-discussions-components-files/902/PI-controller-equations.pdf](https://e2e.ti.com/cfs-file/\_\_key/communityserver-discussions-components-files/902/PI-controller-equations.pdf)

