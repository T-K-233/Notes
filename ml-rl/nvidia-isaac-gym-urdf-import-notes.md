# NVIDIA Isaac Gym URDF Import Notes





```bash
Adding actor 'humanoid-urdf/gym/urdf/robot.urdf' to env 0
Segmentation fault (core dumped)
```

This error could due to multiple reasons, and here's an incomplete checklist

* [ ] Make sure there are \<collision> tags in the link
* [ ] Make sure mesh filename are correct
* [ ] Make sure there is at least one DoF



