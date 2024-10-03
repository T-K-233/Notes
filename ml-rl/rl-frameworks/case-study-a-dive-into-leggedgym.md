# Case Study: A Dive Into LeggedGym

## Code Organization

The majority of the logic is implemented in the `legged_gym/env/base/legged_robot.py` file. The code can be partitioned into the following five sections:

#### 1. Environment creation

How to create the physics environment

#### 2. Adding the agent asset

How to initialize the agent asset (urdf) in the environment, dealing with initial position, joint characteristics etc.

#### 3. Reward design

How to formulate reward to enable efficient learning

#### 4. Exploration mechanism

How to control the aggressiveness of the policy to explore new policy spaces

#### 5. Driving the simulation

How to record the training process and keep the most optimal policy





<figure><img src="../../.gitbook/assets/image (225).png" alt=""><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/image (226).png" alt=""><figcaption></figcaption></figure>















































## The entry function

play.py and train.py

interacts with `task_registry` module



## Step 1: Parse command line arguments

<table data-full-width="true"><thead><tr><th width="186">name</th><th width="71">type</th><th width="88">default</th><th>help</th></tr></thead><tbody><tr><td>task</td><td>str</td><td>go2</td><td>Resume training or start testing from a checkpoint. Overrides config file if provided.</td></tr><tr><td>resume</td><td>bool</td><td>False</td><td>Resume training from a checkpoint</td></tr><tr><td>experiment_name</td><td>str</td><td></td><td>Name of the experiment to run or load. Overrides config file if provided.</td></tr><tr><td>run_name</td><td>str</td><td></td><td>Name of the run. Overrides config file if provided.</td></tr><tr><td>load_run</td><td>str</td><td></td><td>Name of the run to load when resume=True. If -1: will load the last run. Overrides config file if provided.</td></tr><tr><td>checkpoint</td><td>int</td><td></td><td>Saved model checkpoint number. If -1: will load the last checkpoint. Overrides config file if provided.</td></tr><tr><td>headless</td><td>bool</td><td>False</td><td>Force display off at all times</td></tr><tr><td>horovod</td><td>bool</td><td>False</td><td>Use horovod for multi-gpu training</td></tr><tr><td>rl_device</td><td>str</td><td>cuda:0</td><td>Device used by the RL algorithm, (cpu, gpu, cuda:0, cuda:1 etc..)</td></tr><tr><td>num_envs</td><td>int</td><td></td><td>Number of environments to create. Overrides config file if provided.</td></tr><tr><td>seed</td><td>int</td><td></td><td>Random seed. Overrides config file if provided.</td></tr><tr><td>max_iterations</td><td>int</td><td></td><td>Maximum number of training iterations. Overrides config file if provided.</td></tr></tbody></table>



The arguments are then sent to isaacgym to parse

input (custom\_parameters):&#x20;

```python
[
  {'name': '--task', 'type': <class 'str'>, 'default': 'go2', 'help': 'Resume training or start testing from a checkpoint. Overrides config file if provided.'}, 
  {'name': '--resume', 'action': 'store_true', 'default': False, 'help': 'Resume training from a checkpoint'}, 
  {'name': '--experiment_name', 'type': <class 'str'>, 'help': 'Name of the experiment to run or load. Overrides config file if provided.'}, 
  ...
  {'name': '--max_iterations', 'type': <class 'int'>, 'help': 'Maximum number of training iterations. Overrides config file if provided.'}
  ]
```

output (args):

```python
Namespace(
  checkpoint=None, 
  compute_device_id=0, 
  experiment_name=None, 
  flex=False, 
  graphics_device_id=0, 
  headless=False, 
  horovod=False, 
  load_run=None, 
  max_iterations=None, 
  num_envs=4, 
  num_threads=0, 
  physics_engine=SimType.SIM_PHYSX, 
  physx=False, 
  pipeline='gpu', 
  resume=False, 
  rl_device='cuda:0', 
  run_name=None, 
  seed=None, 
  sim_device='cuda:0', 
  sim_device_type='cuda', 
  slices=0, 
  subscenes=0, 
  task='g1_leg', 
  use_gpu=True, 
  use_gpu_pipeline=True
  )
```



## Step 2: Make envrionment

uses `task_registry.make_env()` function

this function finds the corresponding `VecEnv` instance and the configurations (`LeggedRobotCfg` for base, and `LeggedRobotCfgPPO` for training) in the registered environments.

A custom environment can be registered using the function

{% code overflow="wrap" %}
```python
task_registry.register(name: str, task_class: VecEnv, env_cfg: LeggedRobotCfg, train_cfg: LeggedRobotCfgPPO)
```
{% endcode %}



The registation happens in `envs/__init__.py` file.

By default, the environment does not need to be changed. It's all LeggedRobot environment. Only the configurations need to be changed for different robot training environments.



There are several files that defines the fundamental components.

`envs/base/legged_robot.py`

this file defines the `LeggedRobot` environment. It performs the following tasks:

* define how reward and observations are computed, including the definition of each reward terms in `_reward_<term>()`.
* defines the joint level PID controller to map from actions to raw torques in `_compute_torques()`
* initialize the tensor buffers that can be loaded on GPU.
* loads the URDF asset into the environment.
* implements the step() function.



### Environment stepping logic

clips the input actions argument and send it to GPU

render the current scene

for each rendering FPS, perform `decimation` number of physics updates:

&#x20;   compute torques from the input action and set it in sim

&#x20;   step the physics for one iteration

&#x20;   in test mode, delay for correspoinding amout of time to ensure rendering synced to real-time

&#x20;   updates the dof\_state tensor

calculates the root pose and velocity

resamples command for the next step

push robot, if applied

checks termination condition

computes reward

resets the environment instance that is terminated

compute observations for the next step

update the action, dof\_vel, and root\_vel history buffer



## Step 3: Make algorithm runner

uses `task_registry.make_alg_runner()` function

It sets up the tensorboard logging, and initializes and returns the `OnPolicyRunner` class instance from rsl\_rl library.



The rsl\_rl library currently only supports PPO algorithm.

### PPO algorithm

Proximal Policy Optimization (PPO) is an actor-critic method, meaning it utilizes both a policy network (the actor) and a value network (the critic). The actor is responsible for selecting actions, while the critic estimates the value function (expected reward). PPO aims to improve the policy by taking small steps to ensure stable and reliable updates.



### PPO training flow

1. **Initialize networks and storage buffer**.
2. **Collect trajectories** using the current policy.
3. **Compute advantages** and **targets**.
4. **Update policy (actor) network** using the clipped objective.
5. **Update value (critic) network** by minimizing value loss.
6. **Repeat** until convergence.



#### **1: Initialize Networks and Storage Buffer**

Upon creation of the OnPolicyRunner instance, it gets and initializes the Algorithm class specified by `self.cfg["policy_class_name"]`

By default, the `ActorCritic` policy is used, which contains the actor MLP, for calculating the action based on observation, and critic MLP, for calculating the value function given all the observations, including the privileged ones.

The storage buffer `RolloutStorage` is also initialized at this stage which records the `Transitions`.

```python
class Transition:
    def __init__(self):
        self.observations = None
        self.critic_observations = None
        self.actions = None
        self.rewards = None
        self.dones = None
        self.values = None
        self.actions_log_prob = None
        self.action_mean = None
        self.action_sigma = None
        self.hidden_states = None
```

Lastly, it also performs an environment reset to prepare for the first training step.



#### **2: Collect data**

For each training loop, a policy rollout is first performed.

The current policy is ran for `num_steps_per_env` steps to collect trajectories using the `act()` function.

Note that inside the `ppo_runner.act()` function, it both invokes the `actor_critic.act(obs)` function to get the desired actions, and the `actor_critic.evaluate(criitc_obs)`  function to get the value $$V(s)$$ of the current transition. The value is then also stored in the transition buffer. This saves the effort of calling critic again when calculating the advantage.



#### **3: Compute advantages and targets**

The advantage $$A(s, a)$$ is calculated in the `compute_returns()`  function, provided by the rollout buffer.

The  Generalized Advantage Estimation (GAE) method is used to calculate the advantage:

$$
\delta = r_t + \gamma V(s_{t+1}) - V(s_t)
$$

$$
A(s, a) = A_t = \delta + \gamma^t \lambda A_{t+1}
$$



Then, the target value is computed:

$$
R_t = A_t + V(s_t)
$$



```python
def compute_returns(self, last_values, gamma, lam):
    advantage = 0
    for step in reversed(range(self.num_transitions_per_env)):
        if step == self.num_transitions_per_env - 1:
            next_values = last_values
        else:
            next_values = self.values[step + 1]
        next_is_not_terminal = 1.0 - self.dones[step].float()
        delta = self.rewards[step] + next_is_not_terminal * gamma * next_values - self.values[step]
        advantage = delta + next_is_not_terminal * gamma * lam * advantage
        self.returns[step] = advantage + self.values[step]

    # Compute and normalize the advantages
    self.advantages = self.returns - self.values
    self.advantages = (self.advantages - self.advantages.mean()) / (self.advantages.std() + 1e-8)
```



**4: Update actor network**

\<TODO>



#### **5: Update value (critic) network**

\<TODO>







## Step 4: Train policy / Get inference policy

uses either the `ppo_runner.learn(num_learning_iterations, init_at_random_ep_len)` or the `ppo_runner.get_inference_policy(device)` function





















