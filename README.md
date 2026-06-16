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

### Motion Container (Robot Control)

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

### Physical AI Tools (Recording & Training & Inference)

One-time host setup:

```bash
# NVIDIA Container Runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Unix socket directory
sudo mkdir -p /var/run/robotis/agent_sockets/physical_ai_server
sudo chmod 777 /var/run/robotis/agent_sockets/physical_ai_server

# lerobot submodule (clone with recurse-submodules to skip this)
git submodule update --init --recursive -- physical_ai_tools

# Required directories for physical_ai_tools compose
mkdir -p physical_ai_tools/docker/huggingface physical_ai_tools/docker/workspace
```

Start the Physical AI manager and server with the OMX overrides:

```bash
OMX_WS=$PWD docker compose \
  -f physical_ai_tools/docker/docker-compose.yml \
  -f docker/physical_ai_tools_compose.yaml \
  up -d --force-recreate
```

If the ROS server nodes are not listed after startup, launch the server inside
the container:

```bash
docker exec -d -e ROS_DOMAIN_ID=30 physical_ai_server bash -lc \
'source /root/ros2_ws/install/setup.bash && ros2 launch physical_ai_server physical_ai_server_bringup.launch.py'
```

Verify that the server sees the OMX cameras:

```bash
docker exec physical_ai_server bash -lc \
'source /root/ros2_ws/install/setup.bash && ros2 node list | grep -E "physical_ai_server|rosbridge|web_video|camera"'
```

#### Record & Train

Use the Physical AI Tools web UI at `http://localhost` for recording and
training. Select robot type `omx_f` in the UI before recording or checking
cameras.

#### Inference

Download the inference policy once:

```bash
docker exec -it physical_ai_server bash -lc \
'huggingface-cli download JIUUOO/omx_act_task2_full_d50_s10k_seed1000 \
  --local-dir /root/.cache/huggingface/hub/omx_act_task2 \
  --local-dir-use-symlinks False'
```

Start inference with robot type `omx_f_infer`:

```bash
docker exec -e ROS_DOMAIN_ID=30 physical_ai_server bash -lc "
source /root/ros2_ws/install/setup.bash &&
ros2 service call /set_robot_type physical_ai_interfaces/srv/SetRobotType \
  \"{robot_type: 'omx_f_infer'}\" &&
ros2 service call /task/command physical_ai_interfaces/srv/SendCommand \
  \"{command: 2, task_info: {
    task_name: 'inference',
    task_instruction: [''],
    policy_path: '/root/.cache/huggingface/hub/omx_act_task2',
    fps: 30,
    warmup_time_s: 3,
    record_inference_mode: false
  }}\"
"
```

Stop inference with `FINISH`:

```bash
docker exec -e ROS_DOMAIN_ID=30 physical_ai_server bash -lc "
source /root/ros2_ws/install/setup.bash &&
ros2 service call /task/command physical_ai_interfaces/srv/SendCommand \
  \"{command: 6}\"
"
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
