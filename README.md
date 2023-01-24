# ma_demos

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This package is a collection of all setups used for demonstration purposes for my master thesis.

### Overview of demonstrators

* *Two_distributed_rrbots*: Consists of three robots. Each robots consists of 3 segments connected by 2 revolt revolution joints. All three robots are controlled by the central controller manager. However two of the rrbots are started in separate namespaces and are managed by a sub controller manager respectively. Those register at the central controller manager.

* *kuka_kr_16*: Consist of a basic demonstration of the ros2_control concepts on the kuka kr 16 robot (forward_position_controller and joint_trajectory_controller).

## How to use:
If you are just interested how to start the demos themselves have a look below under the [quickstart](https://github.com/StoglRobotics-forks/ma_demos#quickstart) section.
### Installation:
1. Update system:
    ```
    sudo apt update
    sudo apt upgrade
    ```
2. Install ROS 2 Rolling:
  * a) Either setup a Docker container with Rolling installed using [RosTeamWS](https://rtw.stoglrobotics.de/master/use-cases/operating_system/create_setup_workspace.html#docker-workspace).
 If you use this method, don't forget to switch to your docker container **before continuing** with the `rtw_switch_to_docker` command.
    
  * b) Or [install ROS 2 Rolling](https://docs.ros.org/en/rolling/Installation.html) directly on your computer.
3. Make sure `colcon`and `vcs` are installed:
    ```
    sudo apt install python3-colcon-common-extensions python3-vcstool
    ```
4. Setup new workspace (_If you used [RosTeamWS](https://rtw.stoglrobotics.de/master/use-cases/operating_system/create_setup_workspace.html) and docker you can skip this step._):
    ```
    mkdir -p ~/workspace/rolling_ws/src  # or go to an existing one
    ```
5. Clone this repo:
    ```
    cd ~/workspace/rolling_ws/src
    git clone git@github.com:StoglRobotics-forks/ma_demos.git 
    (Or gh repo clone StoglRobotics-forks/ma_demos)
    ```
6. Make sure your base workspace is sourced and update dependencies:
   ```
   source /opt/ros/rolling/setup.bash # source ws
   rosdep update                      # update dependencies
   ```
7. Get the relevant packages and install all additional dependencies:
   ```
   cd ~/workspace/rolling_ws/         # or your workspace base direcotry
   vcs import src --skip-existing --input src/ma_demos/ma_demos.rolling.repos 
   rosdep install --ignore-src --from-paths src -y -r
   ```
8. Finally compile everything:
   ```
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

### Quickstart:
After you hav successfully built the project you can then start the demos as shown below.

* *Two_distributed_rrbots*:
    Open first terminal and execute:
    ```
    ros2 launch two_distributed_rrbots_bringup two_distributed_rrbots.launch.py 
    ```
    Then open the second terminal and execute:
    ```
    ros2 launch two_distributed_rrbots_bringup test_two_distributed_rrbots_forward_position_controller.launch.py
    ```
    
* *kuka_kr_16*:
    Open first terminal and execute:
    ```
    ros2 launch kuka_kr16_2_bringup kuka_kr16_2.launch.py 
    ```
    Then open the second terminal and execute:
    ```
    ros2 launch kuka_kr16_2_bringup test_kuka_kr16_2_forward_position_controller.launch.py
    ```
    Or
    ```
    ros2 launch kuka_kr16_2_bringup test_kuka_kr16_2_joint_trajectory_controller.launch.py
    ```
