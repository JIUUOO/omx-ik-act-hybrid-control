# OMX IK-ACT Hybrid Control

Hybrid manipulation workspace for Open Manipulator-X using ROS 2 Jazzy.

## Setup

Clone the repository with its submodules:

```bash
git clone --recursive https://github.com/JIUUOO/omx-ik-act-hybrid-control.git
cd omx-ik-act-hybrid-control
```

Connect both OpenRB devices and check their device paths and serial numbers:

```bash
for device in /dev/ttyACM*; do
  echo "$device"
  udevadm info --query=property --name="$device" | grep '^ID_SERIAL_SHORT='
done
```

Update the leader and follower serial numbers in
`config/99-omx-openrb.rules`, then install the udev rules:

```bash
./scripts/install_udev_rules.sh
```

Reconnect the OpenRB devices, then verify that both device links exist:

```bash
ls -l /dev/omx_leader /dev/omx_follower
```

## Docker

Build and start the motion container:

```bash
docker compose -f docker/compose.yaml up -d --build motion
docker compose -f docker/compose.yaml exec motion bash
```

Inside the container, install dependencies and build the workspace:

```bash
rosdep update
apt-get update
rosdep install --from-paths /root/ros2_ws/src --ignore-src -r -y --rosdistro jazzy
omxbuild
```

## Leader-Follower Teleoperation

In the motion container, launch the follower:

```bash
ros2 launch open_manipulator_bringup omx_f_follower_ai.launch.py \
  port_name:=/dev/omx_follower \
  init_position:=false
```

In another host terminal, enter the motion container and launch the leader:

```bash
docker compose -f docker/compose.yaml exec motion bash
omxws

ros2 launch open_manipulator_bringup omx_l_leader_ai.launch.py \
  port_name:=/dev/omx_leader
```
