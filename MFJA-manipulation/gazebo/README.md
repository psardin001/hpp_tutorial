# Gazebo demonstration

This optional demonstration loads the Room 315 Stäubli scene and replays the
reference arm and pick-and-place trajectories. The HPP plan remains the source
of the robot and box motion.

From `MFJA-manipulation`, start the scene:

```bash
./gazebo/run.sh launch
```

When the scene is ready, use a second terminal:

```bash
./gazebo/run.sh execute
```

The launcher uses ROS domain 42 with local-host discovery. Set
`GAZEBO_ROS_DOMAIN_ID` in both terminals to select another domain.
