# Instructions for Running Code Using Docker

This guide provides instructions for running the Geodude stack in a containerized setting using Docker.

## Building the Docker Image

First, build the Docker image:
```bash
docker build -t isaacsim_humble_hardware -f docker/dockerfile.isaacsim4.humble.hardware .
```

Add local to xhost:
```bash
xhost local:root
```

## Running the Docker Container

Run the Docker container with the following command:
```bash
docker run --name isaac-sim-humble-hardware --entrypoint bash -it --privileged -e DISPLAY=unix${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
-e "PRIVACY_CONSENT=Y" \
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
-v /home:/home \
-v /dev:/dev \
isaacsim_humble_hardware
```

## Testing the Mock Interface

### Terminal 1:

Run the following command:
```bash
docker run --name isaac-sim-humble-hardware --entrypoint bash -it --privileged -e DISPLAY=unix${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
-e "PRIVACY_CONSENT=Y" \
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
-v /home:/home \
-v /dev:/dev \
isaacsim_humble_hardware
```

Navigate to the workspace:
```bash
cd PATH_TO_GEODUDE_WS  # Replace with your actual path, e.g., /home/helen/geodude_ws
```

Build the workspace:
```bash
colcon build
source install/setup.bash
```

Launch the hardware simulation:
```bash
ros2 launch geodude_hardware geodude_hardware.launch.py 'sim:=mock'
```

### Terminal 2:

List the container ID and enter the container:
```bash
docker exec -it isaac-sim-humble-hardware bash
```

Navigate to the workspace:
```bash
cd PATH_TO_GEODUDE_WS
source install/setup.bash
```

Launch the MoveIt demo:
```bash
ros2 launch geodude_moveit demo.launch.py
```

### Terminal 3:

Enter the container:
```bash
docker exec -it isaac-sim-humble-hardware bash
```

Navigate to the workspace:
```bash
cd PATH_TO_GEODUDE_WS
source install/setup.bash
```

Activate the controllers:
```bash
ros2 control set_controller_state right_arm_controller active
ros2 control set_controller_state joint_state_broadcaster_right_arm active
```

## Testing the Isaac-Sim Interface

### Terminal 1:

Run the following command:
```bash
docker run --name isaac-sim-humble-hardware --entrypoint bash -it --privileged -e DISPLAY=unix${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
-e "PRIVACY_CONSENT=Y" \
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
-v /home:/home \
-v /dev:/dev \
isaacsim_humble_hardware
```

Navigate to the workspace:
```bash
cd PATH_TO_GEODUDE_WS  # Replace with your actual path, e.g., /home/helen/geodude_ws
```

Build the workspace:
```bash
colcon build
source install/setup.bash
```

Launch the hardware simulation:
```bash
ros2 launch geodude_hardware geodude_hardware.launch.py 'sim:=isaac'
```

### Terminal 2:

List the container ID and enter the container:
```bash
docker exec -it isaac-sim-humble-hardware bash
```

Navigate to the workspace:
```bash
cd PATH_TO_GEODUDE_WS
source install/setup.bash
```

Launch the MoveIt demo:
```bash
ros2 launch geodude_moveit demo.launch.py
```

### Terminal 3:

Enter the container:
```bash
docker exec -it isaac-sim-humble-hardware bash
```

Navigate to the workspace:
```bash
cd PATH_TO_GEODUDE_WS
source install/setup.bash
```

Activate the controllers:
```bash
ros2 control set_controller_state right_arm_controller active
ros2 control set_controller_state joint_state_broadcaster_right_arm active
```

### Terminal 4:

Enter the container:
```bash
docker exec -it isaac-sim-humble-hardware bash
```

Launch Isaac Interface
```bash
cd /PATH/TO/geodude_ws/src/geodude_ros2/geodude_simulation/isaac_geodude/launch
./python.sh isaac_moveit.py /PATH/TO/geodude_ws/src/geodude_ros2/geodude_simulation/geodude_latest/geodude.usd
```