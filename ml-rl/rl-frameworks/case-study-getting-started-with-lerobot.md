# Case Study: Getting Started with LeRobot

{% embed url="https://github.com/huggingface/lerobot/blob/main/examples/7_get_started_with_real_robot.md" %}



```bash
conda activate lerobot
```





Grant permission

```bash
sudo nano /etc/udev/rules.d/50-lerobot.rules
```

{% code title="50-lerobot.rules" overflow="wrap" %}
```bash
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", GROUP="users", MODE="0666"
```
{% endcode %}

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```







Teleoperation demo

```bash
python lerobot/scripts/control_robot.py teleoperate \
  --robot-path lerobot/configs/robot/koch.yaml
```





Login to hugging face

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
HF_USER=$(huggingface-cli whoami | head -n 1)
echo $HF_USER
```



record dataset

```bash
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/koch.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/koch_test \
  --tags tutorial \
  --num-episodes 2 \
  --warmup-time-s 5 \
  --episode-time-s 30 \
  --reset-time-s 30
```



full 50 episodes!

```bash
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/koch.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/koch_place_block \
  --tags tutorial \
  --num-episodes 50 \
  --warmup-time-s 2 \
  --episode-time-s 30 \
  --reset-time-s 10
```





Visualize

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --root data \
  --repo-id ${HF_USER}/koch_place_block
```



Replay

```bash
python lerobot/scripts/control_robot.py replay \
  --robot-path lerobot/configs/robot/koch.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/koch_place_block \
  --episode 0
```





Training!!

```bash
DATA_DIR=data python lerobot/scripts/train.py \
  dataset_repo_id=${HF_USER}/koch_place_block \
  policy=act_koch_real \
  env=koch_real \
  hydra.run.dir=outputs/train/act_koch_place_block \
  hydra.job.name=act_koch_place_block \
  device=cuda \
  wandb.enable=true
```



upload trained model

{% code overflow="wrap" %}
```bash
huggingface-cli upload ${HF_USER}/act_koch_place_block   outputs/train/act_koch_place_block/checkpoints/last/pretrained_model
```
{% endcode %}



eval

```bash
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/koch.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/eval_koch_place_block \
  --tags tutorial eval \
  --warmup-time-s 5 \
  --episode-time-s 30 \
  --reset-time-s 30 \
  --num-episodes 10 \
  -p outputs/train/act_koch_place_block/checkpoints/last/pretrained_model
```









Teleop Script

```python
import tqdm
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.motors.dynamixel import TorqueMode
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera


leader_port = "/dev/ttyACM0"
follower_port = "/dev/ttyACM1"

leader_arm = DynamixelMotorsBus(
    port=leader_port,
    motors={
        # name: (index, model)
        "shoulder_pan": (1, "xl330-m077"),
        "shoulder_lift": (2, "xl330-m077"),
        "elbow_flex": (3, "xl330-m077"),
        "wrist_flex": (4, "xl330-m077"),
        "wrist_roll": (5, "xl330-m077"),
        "gripper": (6, "xl330-m077"),
    },
)

follower_arm = DynamixelMotorsBus(
    port=follower_port,
    motors={
        # name: (index, model)
        "shoulder_pan": (1, "xl430-w250"),
        "shoulder_lift": (2, "xl430-w250"),
        "elbow_flex": (3, "xl330-m288"),
        "wrist_flex": (4, "xl330-m288"),
        "wrist_roll": (5, "xl330-m288"),
        "gripper": (6, "xl330-m288"),
    },
)

robot = ManipulatorRobot(
    robot_type="koch",
    leader_arms={"main": leader_arm},
    follower_arms={"main": follower_arm},
    calibration_dir=".cache/calibration/koch",
    cameras={
        "laptop": OpenCVCamera(0),
    },
)

robot.connect()

follower_arm.write("Torque_Enable", TorqueMode.ENABLED.value)

# # leader_pos = robot.leader_arms["main"].read("Present_Position")
# # follower_pos = robot.follower_arms["main"].read("Present_Position")

# # print(leader_pos)
# # print(follower_pos)

seconds = 120
frequency = 200

for _ in tqdm.tqdm(range(seconds*frequency)):
    leader_pos = robot.leader_arms["main"].read("Present_Position")
    robot.follower_arms["main"].write("Goal_Position", leader_pos)

    # observation, action = robot.teleop_step(record_data=True)
    # print(observation["observation.images.laptop"].shape)
    # print(observation["observation.images.phone"].shape)
    # print(observation["observation.images.laptop"].min().item())
    # print(observation["observation.images.laptop"].max().item())


follower_arm.write("Torque_Enable", TorqueMode.DISABLED.value)

robot.disconnect()

```





