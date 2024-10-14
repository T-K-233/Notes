# Case Study: A Dive Into IsaacLab



## Basic File Hierarchy

Most of the environment related config files are in `source/extensions` folder.&#x20;

The folder `source/standalone` provides a set of demo programs, tools, and the entry script to the manager-based environments that can be invoked user.





## Environment Definitions





## Task definitions

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













## Fixing the robot in-place

Sometimes when debugging, we wish to pin the robot in a fixed location. This can be achieved by changing the following attribute:

```python
class EnvCfg():
    def __post_init__(self):
        super().__post_init__()
        # assume self.scene.robot is already defined
        self.scene.robot.spawn.articulation_props.fix_root_link = True

```





















## Adding Camera View

```python
import omni.ui as ui

...

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    # im = plt.imshow(np.zeros((480, 640, 1), dtype=np.float32))
    # # set scale to 1.0
    # im.set_clim(0, 10)
    # plt.colorbar(im)
    # cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("Depth Image", 640, 480)

    depth_window = ui.Window("Depth Image", width=640, height=480)
    with depth_window.frame:
        bytes_provider = ui.ByteImageProvider()
        bytes_provider.set_bytes_data(np.zeros((480, 640, 1), dtype=np.float32).flatten().data, [480, 640], ui.TextureFormat.R32_SFLOAT)
        depth_image_widget = ui.ImageWithProvider(bytes_provider)  # Create an image widget



```

```python

    while simulation_app.is_running():
        # print information from the sensors
        # print("-------------------------------")
        # print(scene["camera"])
        # print("Received shape of rgb   image: ", scene["camera"].data.output["rgb"].shape)

        # visualize depth image
        depth_image = scene["camera"].data.output["distance_to_image_plane"]
        depth_image = depth_image[0, ...].cpu().numpy().astype(np.float32)

        depth_image /= 1000.0
        
        # im.set_data(depth_image)
        # plt.pause(0.001)  # Allow the plot to update

        # cv2.imshow("Depth Image", depth_image)
        # cv2.waitKey(1)
        bytes_provider.set_bytes_data(depth_image.flatten().data, [480, 640], ui.TextureFormat.R32_SFLOAT)

        print("depth_image.shape: ", depth_image.shape)

        # print("Received shape of depth image: ", scene["camera"].data.output["distance_to_image_plane"].shape)
        # print("-------------------------------")
        # print(scene["height_scanner"])
        # print("Received max height value: ", torch.max(scene["height_scanner"].data.ray_hits_w[..., -1]).item())
        # print("-------------------------------")
        # print(scene["contact_forces"])
        # print("Received max contact force of: ", torch.max(scene["contact_forces"].data.net_forces_w).item())


```



{% embed url="https://github.com/Toni-SM/semu.data.visualizer/blob/main/src/semu.data.visualizer/semu/data/visualizer/visualizer.py#L285" %}











