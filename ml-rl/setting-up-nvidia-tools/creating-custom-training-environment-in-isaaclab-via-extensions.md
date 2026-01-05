# Creating Custom Training Environment in IsaacLab via Extensions

As mentioned in the "Dive Into IsaacLab" article, the codebase of IsaacLab is significantly larger than other RL frameworks.

{% content-ref url="../rl-frameworks/case-study-a-dive-into-isaaclab.md" %}
[case-study-a-dive-into-isaaclab.md](../rl-frameworks/case-study-a-dive-into-isaaclab.md)
{% endcontent-ref %}



It provides a nice abstraction of configuration fragments using the Config class and the notion of Managers. However, this makes editing the environment and implementing custom training logic harder. Jumping across directory structures to access config fragments is common and annoying.



Fortunately, IsaacLab also supports [Extensions](https://isaac-sim.github.io/IsaacLab/source/overview/developer-guide/template.html), which allows us to isolate out the custom scripts into a separate repository. In this article, we will go over how to create such extension and achieve custom training logic.



## Cloning the extension template

```bash
git clone https://github.com/isaac-sim/IsaacLabExtensionTemplate.git
cd ./IsaacLabExtensionTemplate/
```



## Rename the template

We can rename the template to a custom extension name. Here, we will use `g1_demo` as an example:

```bash
python scripts/rename_template.py g1_demo
```

<figure><img src="../../.gitbook/assets/image (2) (1) (1) (1) (1) (1) (1) (1).png" alt=""><figcaption></figcaption></figure>

After executing this command, all the `ext_template` text inside this repo should be replace to `g1_demo`.



## Configure the tasks

We start off by copying the scripts directly from upstream IsaacLab



There's a few difference between the scripts used in IsaacLab versus in the Extension.

{% code title="velocity_env_cfg.py" %}
```python
# IsaacLab
import omni.isaac.lab_tasks.manager_based.locomotion.velocity.mdp as mdp

# Extension
import g1_demo.tasks.locomotion.velocity.mdp as mdp
```
{% endcode %}



{% code title="train.py & play.py" %}
```bash
# IsaacLab
import omni.isaac.lab_tasks  # noqa: F401

# Extension
import g1_demo.tasks  # noqa: F401
```
{% endcode %}













