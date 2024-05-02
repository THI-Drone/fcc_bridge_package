# FCC bridge
Node with mavlink access. It verifies and sends waypoints requested by other nodes. Also handles RTH and landing.

## Setup on the Raspberry Pi

1. Clone the **thi-drone-ws** repo to the Pi
1. Enter the `.devcontainer` directory and build the docker container. Don't forget to set the username.
    ```
    docker build -t devcontainer --build-arg="USERNAME=admin"
    ```
1. Run the container with the following args:
    ```
    docker run -ti -d --device=/dev/serial0 -ti -v ~/thi-drone-ws:/home/ws --name thi_drohne devcontainer bash
    ```
1. Attach to the container:
    ```
    docker attach thi_drohne
    ```
1. Install mavsdk:
    ```
    wget https://github.com/mavlink/MAVSDK/releases/download/v2.9.1/libmavsdk-dev_2.9.1_debian12_arm64.deb
    sudo dpkg -i libmavsdk-dev_2.9.1_debian12_arm64.deb
    ```
1. Create a parameter file, for example `uav_params.yaml` to set the `UAV_ID` ROS parameter (here I set it to team red):
    ```
    fcc_bridge:
      ros__parameters:
        UAV_ID: "UAV_TEAM_RED"
    ```
1. Build the package:
    ```
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --package-up-to fcc_bridge
    ```
1. Become **root** to have permissions to write the serial device:
    ```
    sudo -i
    cd /home/ws
    source /opt/ros/humble/setup.bash
    . install/setup.bash
    ```
1. Finally run the node:
    ```
    ros2 run fcc_bridge fcc_bridge --ros-args --params-file /home/ws/uav_params.yaml
    ```