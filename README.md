# ma_demos

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This package is a collection of all setups used for demonstration purposes for my master thesis.

### Overview of demonstrators

* *Two_distributed_rrbots*: Consists of three robots. Each robots consists of 3 segments connected by 2 revolt revolution joints. All three robots are controlled by the central controller manager. However two of the rrbots are started in separate namespaces and are managed by a sub controller manager respectively. Those register at the central controller manager.

* *kuka_kr_16*: Consist of a basic demonstration of the ros2_control concepts on the kuka kr 16 robot (forward_position_controller and joint_trajectory_controller).
