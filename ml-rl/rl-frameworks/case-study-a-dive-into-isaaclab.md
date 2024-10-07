# Case Study: A Dive Into IsaacLab



## Basic File Hierarchy

Most of the environment related config files are in `source/extensions` folder.&#x20;

The folder `source/standalone` provides a set of demo programs, tools, and the entry script to the manager-based environments that can be invoked user.



### Task definitions

The tasks are defined in the `source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based` folder.

The interface to these RL frameworks are defined in `source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/wrappers`



We define the task-specific configuration and parameters.

A change compared to LeggedGym is that the registration of task is moved to each robot's own `__init__.py` file.



The environment is defiend as `RslRlVecEnvWrapper`, where it wraps around the `ManagerBasedRLEnv` class.



For config file, it follows this inheritance logic

ManagerBasedEnvCfg -> ManagerBasedRLEnvCfg -> LocomotionVelocityRoughEnvCfg -> G1RoughEnvCfg

#### ManagerBasedEnvCfg

it defines the following config pieces:

viewer: ViewerCfg -&#x20;

sim: SimulationCfg -&#x20;

ui\_window\_class\_type -&#x20;

decimation -&#x20;

scene: InteractiveSceneCfg -&#x20;

observations - observation space settings

actions - action space settings

events: DefaultEventManagerCfg - handles reset events



#### ManagerBasedRLEnvCfg

is\_finite\_horizon: bool

episode\_length\_s: float - duration of an episode in seconds, `episode_length_steps = ceil(episode_length_s / (decimation_rate * physics_time_step))`

rewards: RewardManager

terminations: TerminationManager

curriculum: CurriculumManager

commands: CommandManager



#### LocomotionVelocityRoughEnvCfg

It instanciates the managers upon initialization, and provides a custom `__post_init__` routine











