This package includes the launch files for the first scenario. This scenario test the concept of having one central controller manager with a controller running (joint trajectory controller) and  two distributed sub controller managers with the drivers and communication to the robot.

# Scenario one
The `main_controller_no_robot.launch.py` is the before mentioned scenario. One central controller manager with a controller and two sub controller managers with no controllers but the robot drivers running. Central controller manager communicates with hardware via distributed handles and moves the robots.

## How to launch
1. Start central controller manager: `ros2 launch two_distributed_kukas_bringup main_controller_no_robot.launch.py`
2. Start the two distributed sub controller managers:
    * `ros2 launch two_distributed_kukas_bringup sub_1.launch.py use_mock_hardware:=true`
    * `ros2 launch two_distributed_kukas_bringup sub_2.launch.py use_mock_hardware:=true`
    
    For the real robots set the correct ip address of your computer with the `listen_ip_address:=` parameter:
    * `ros2 launch two_distributed_kukas_bringup sub_1.launch.py listen_ip_address:=172.20.19.102`
    * `ros2 launch two_distributed_kukas_bringup sub_2.launch.py listen_ip_address:=172.20.19.101`
3. Start controller of central controller manager and rviz2: `ros2 launch two_distributed_kukas_bringup controllers_and_rviz.launch.py` 
4. Publish the goals for the joint trajectory controller: `ros2 launch two_distributed_kukas_bringup joint_trajectory_controller_goals_publisher.launch.py goals_file:=no_central_two_distributed_kuka_6dof_joint_trajectory_controller_goals.yaml`

# Sub scenario A
The `main_controller_with_robot.launch.py` is a small modification of the before mentioned scenario.The one central controller manager now handles the drivers of one robot as well. There is only one sub controller manager with no controllers but the robot driver running. Central controller manager communicates with hardware via distributed handles and moves the robots.

## How to launch
1. Start central controller manager:
    * `ros2 launch two_distributed_kukas_bringup main_controller_with_robot.launch.py use_mock_hardware:=true`

    For the real robots set the correct ip address of your computer with the `listen_ip_address:=` parameter:
    * `ros2 launch two_distributed_kukas_bringup main_controller_with_robot.launch.py listen_ip_address:=172.20.19.101 `
2. Start the distributed sub controller managers. !Only works with sub_1 because of the namespaces and expected joints:
    * `ros2 launch two_distributed_kukas_bringup sub_1.launch.py use_mock_hardware:=true origin:="'0 3 0'"`
    
    For the real robots set the correct ip address of your computer with the `listen_ip_address:=` parameter:
    * `ros2 launch two_distributed_kukas_bringup sub_1.launch.py listen_ip_address:=172.20.19.102 origin:="'0 3 0'"`

3. Start controller of central controller manager and rviz2: `ros2 launch two_distributed_kukas_bringup controllers_and_rviz.launch.py` 
4. Publish the goals for the joint trajectory controller: `ros2 launch two_distributed_kukas_bringup joint_trajectory_controller_goals_publisher.launch.py goals_file:=one_central_one_distributed_kuka_6dof_joint_trajectory_controller_goals.yaml`

Note: aio=all in one.  The `no_central_two_distributed_aio.launch.py` corresponds to scenario one but launches everything at once. Used to test locally with mock_hardware.   
The `one_central_one_distributed_aio.launch.py` corresponds to sub scenario A but launches everything at once. Used to test locally with mock_hardware. 