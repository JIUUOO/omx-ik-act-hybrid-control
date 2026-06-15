# OMX Cyclo Safe Controller

This overlay uses ROBOTIS Cyclo's OMX MoveL controller and collision model
without modifying the `cyclo_control` submodule.

At the configured Home pose, the `link5_0` safety sphere is treated as a
radius-zero point. The normal 50 mm sphere is restored permanently after every
raw collision pair involving that sphere reaches 7 mm clearance.

While startup relaxation is active:

- `joint5` is locked to prevent gripper roll.
- The gripper must remain closed.
- No arm trajectory is published before the first MoveL command.

## Build

```bash
docker compose -f docker/compose.yaml exec motion bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /root/ros2_ws
  colcon build --symlink-install --packages-up-to omx_cyclo_safe_controller
'
```

## Run

Bring the OMX-F to its configured initial Home pose first, then run:

```bash
docker compose -f docker/compose.yaml exec motion bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /root/ros2_ws/install/setup.bash
  ros2 launch omx_cyclo_safe_controller omx_safe_controller.launch.py
'
```

Do not run ROBOTIS's original `omx_movel_controller_node` at the same time.

Check whether the normal safety sphere has been restored:

```bash
ros2 topic echo --once /omx_movel_controller/startup_relaxation_active
```

`true` means startup relaxation is still active. `false` means the normal
collision sphere is active and startup relaxation cannot reactivate until the
controller is restarted.
