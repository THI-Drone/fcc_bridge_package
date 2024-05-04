# FCC bridge
Node with mavlink access. It verifies and sends waypoints requested by other nodes. Also handles RTH and landing.

## Setup in the jmavsim simulator

1. Clone the **thi-drone-ws** repo. In the **same folder, next to** `thi-drone-ws`, clone the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repo.
    ```
    git clone git@github.com:THI-Drone/thi-drone-ws.git
    git clone git@github.com:PX4/PX4-Autopilot.git
    ```
1. Enter the `.devcontainer/general_devcontainer` directory and build the general devcontainer. Don't forget to set your username!
    ```
    # it's important to use the "general_devcontainer" tag!!
    docker build -t general_devcontainer --build-arg="USERNAME=REPLACE_ME"
    ```
1. Open **thi-drone-ws** in VScode and select _Reopen in devcontainer_.
1. When prompted with a selection, choose the FCC bridge container to start the build. _This will take a while._
1. After the build finally finished, open an external terminal to start a second shell for the container. We will compile and run the simulator there:
   ```
   cd /home/PX4-Autopilot
   HEADLESS=1 make px4_sitl jmavsim
   ```
1. While the simulator is compiling, switch back to your VScode terminal. There you can build the package:
    ```
    cd /home/ws
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --package-up-to fcc_bridge
    . install/setup.bash
    ```
1. Create a parameter file, for example `simulator_params.yaml` to set the `UAV_ID` ROS parameter:
    ```
    fcc_bridge:
      ros__parameters:
        UAV_ID: "SIMULATOR"
    ```
1. **After** the simulator finished building and is running, you can finally run the node:
    ```
    ros2 run fcc_bridge fcc_bridge --ros-args --params-file /home/ws/simulator_params.yaml
    ```

> For troubleshooting, refer to the [simulation repo]([/THI-Drone/simulator](https://github.com/THI-Drone/simulation)).

## Setup on the Raspberry Pi

1. Clone the **thi-drone-ws** repo to the Pi
1. Enter the `.devcontainer/general_devcontainer` directory and build the docker container. Don't forget to set the username.
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
