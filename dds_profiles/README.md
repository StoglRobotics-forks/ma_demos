# DDS in ROS2
Simple testing:
Clone the ros2_demos project:
1. Open two terminal.
2. Execute `RMW_IMPLEMENTATION=<rmw_implementation>` e.g.: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
3. In the first terminal: `ros2 run demo_nodes_cpp talker`
4. In the second terminal: `ros2 run demo_nodes_cpp listener`
## Overview of ros2 middlewares: (Rolling default is Fast DDS)
| **Product name**          | **License**                   | **RMW implementation**    |**Status** | **Working?** | **Testing**|
| :-------------------:     | :----------:                  | :----------------------: | :-------------: | :---: |:---: |
| [eProsima **Fast DDS**](https://fast-dds.docs.eprosima.com/en/latest/)         | Apache 2                      | ``RMW_IMPLEMENTATION=rmw_fastrtps_cpp``      | Full support. **Default RMW**. Packaged with binary releases.| YES. out of the box|✓|
| [Eclipse **Cyclone DDS**](https://projects.eclipse.org/projects/iot.cyclonedds)       | Eclipse Public License v2.0   | ``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp``      | Full support. Packaged with binary releases. | YES. binary install |✓|
| [RTI **Connext DDS**](https://www.rti.com/products)           | commercial, research          | ``RMW_IMPLEMENTATION=rmw_connextdds``      | Full support. Support included in binaries, but Connext installed separately. | YES. Need to install correct binary and license |✓|
|[Eclipse **Zenoh**](https://zenoh.io/)  |?|?|?|?|✓|
||||||
| [GurumNetworks **GurumDDS**](https://gurum.cc/index_eng)    | commercial                    |  Not working      | Community support. Support included in binaries, but GurumDDS installed separately. |NO. commercial license needed|X|

RTPS (a.k.a. DDSI-RTPS) is the wire protocol used by DDS to communicate over the network.

In order to use a DDS/RTPS implementation with ROS 2, a “ROS Middleware interface” (a.k.a. rmw interface or just rmw) package needs to be created that implements the abstract ROS middleware interface using the DDS or RTPS implementation’s API and tools

## DDS Useful Resources:

### Different DDS vendors
* [About different ROS 2 DDS/RTPS vendors](https://docs.ros.org/en/rolling/Concepts/About-Different-Middleware-Vendors.html) :  

    Overview of different middle ware and what to considere. 
    
        ROS 2 supports multiple DDS/RTPS implementations because it is not necessarily “one size fits all” when it comes to choosing a vendor/implementation. There are many factors you might consider while choosing a middleware implementation: logistical considerations like the license, or technical considerations like platform availability, or computation footprint.

* [Working with multiple ROS 2 middleware implementations](https://docs.ros.org/en/rolling/How-To-Guides/Working-with-multiple-RMW-implementations.html)

    Overview how to switch between different vendors and implementations.

        Both C++ and Python nodes support an environment variable RMW_IMPLEMENTATION that allows the user to select the RMW implementation to use when running ROS 2 applications.
        The user may set this variable to a specific implementation identifier, such as rmw_cyclonedds_cpp, rmw_fastrtps_cpp, rmw_connextdds, or rmw_gurumdds_cpp.

        Adding RMW implementations to your workspace

        Troubleshooting

        Ensuring use of a particular RMW implementation -> ! Daemon needs to be stopped !

* [DDS implementations](https://docs.ros.org/en/rolling/Installation/DDS-Implementations.html)

    Overview of different vendors and implementations. Examples how to install and setup license on different platforms.

        Once you’ve installed a new DDS vendor, you can change the vendor used at runtime. Todo so set the environment variable: $ export RMW_IMPLEMENTATION=<rmw_version>

***
##  eProsima fast DDS: [Documentation](https://fast-dds.docs.eprosima.com/en/latest/index.html) | [Repo](https://github.com/eProsima/Fast-DDS) | [ROS2-Repo](https://github.com/ros2/rmw_fastrtpsS)


### Installation:
How to instal is described [here](https://docs.ros.org/en/rolling/Installation/DDS-Implementations/Working-with-eProsima-Fast-DDS.html).

### Setting different parameters
* [How to set xml profiles](https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html#xml-profiles)
* [Example how to set tcp as default](https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/)




***
##  Eclipse Cyclone DDS: [Documentation](https://cyclonedds.io/docs/) | [Repo](https://github.com/eclipse-cyclonedds/cyclonedds)


### Installation:
How to instal is described [here](https://docs.ros.org/en/rolling/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html).




***
##  RTI Connext DDS: [Documentation](https://community.rti.com/documentation)


### Installation:
How to instal is described [here](https://docs.ros.org/en/rolling/Installation/DDS-Implementations.html).
 Be aware that additionally you have to install the correct binaries: 
 1. ``sudo apt update && sudo apt install -q -y rti-connext-dds-6.0.1`` 
 2. Binary installation for rolling: ``sudo apt install ros-rolling-rmw-connextdds`` 
 3. Activate license: ``cd /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -``



***
##  Eclipse Zenoh: [Documentation](https://zenoh.io/docs/getting-started/first-app/) | [Repo](https://github.com/eclipse-zenoh)
### Installation:
!!! Zenoh (zenohd) depends on systemd. However systemd is not present by default inside docker. So the following won't work out of the box. You first have to install systemd inside docker !!!  
How to install is described [here](https://zenoh.io/blog/2021-04-28-ros2-integration/) or in more detail in the [github repo](https://github.com/eclipse-zenoh/zenoh-plugin-dds#How-to-install-it)
1. `echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null`
2. `sudo apt update`
3. `sudo apt install zenoh zenoh-plugin-dds zenoh-bridge-dds`.

### Quick Test
### _1 host, 2 ROS domains_
For a quick test on a single host, you can run the `turtlesim_node` and the `turtle_teleop_key` on distinct ROS domains. As soon as you run 2 `zenoh-bridge-dds` (1 per domain) the `turtle_teleop_key` can drive the `turtlesim_node`.  
Here are the commands to run:
  - `ROS_DOMAIN_ID=1 ros2 run turtlesim turtlesim_node`
  - `ROS_DOMAIN_ID=2 ros2 run turtlesim turtle_teleop_key`
  - `./target/release/zenoh-bridge-dds -d 1`
  - `./target/release/zenoh-bridge-dds -d 2`

Notice that by default the 2 bridges will discover each other using UDP multicast.

### _2 hosts, avoiding UDP multicast communication_
By default DDS (and thus ROS2) uses UDP multicast for discovery and publications. But on some networks, UDP multicast is not or badly supported.  
In such cases, deploying the `zenoh-bridge-dds` on both hosts will make it to:
  - limit the DDS discovery traffic, as detailled in [this blog](https://zenoh.io/blog/2021-03-23-discovery/#leveraging-resource-generalisation)
  - route all the DDS publications made on UDP multicast by each node through the zenoh protocol that by default uses TCP.

Here are the commands to test this configuration with turtlesim:
  - on host 1:
    - `ROS_DOMAIN_ID=1 ros2 run turtlesim turtlesim_node`
    - `./target/release/zenoh-bridge-dds -d 1 -l tcp/0.0.0.0:7447`
  - on host 2:
    - `ROS_DOMAIN_ID=2 ros2 run turtlesim turtle_teleop_key`
    - `./target/release/zenoh-bridge-dds -d 2 -e tcp/<host-1-ip>:7447` - where `<host-1-ip>` is the IP of host 1

Notice that to avoid unwanted direct DDS communication, 2 disctinct ROS domains are still used.

### Test with distributed control:
TLDR: Start bridges with `-f` -> `./target/release/zenoh-bridge-dds -d 1 -f`  
By default the bridge doesn't route throught zenoh the DDS discovery traffic to the remote bridges.  
Meaning that, in case you use 2 **`zenoh-bridge-dds`** to interconnect 2 DDS domains, the DDS entities discovered in one domain won't be advertised in the other domain. Thus, the DDS data will be routed between the 2 domains only if matching readers and writers are declared in the 2 domains independently.

This default behaviour has an impact on ROS2 behaviour: on one side of the bridge the ROS graph might not reflect all the nodes from the other side of the bridge. The `ros2 topic list` command might not list all the topics declared on the other side. And the **ROS graph** is limited to the nodes in each domain.

But using the **`--fwd-discovery`** (or `-f`) option for all bridges make them behave differently:
 - each bridge will forward via zenoh the local DDS discovery data to the remote bridges (in a more compact way than the original DDS discovery traffic)
 - each bridge receiving DDS discovery data via zenoh will create a replica of the DDS reader or writer, with similar QoS. Those replicas will serve the route to/from zenoh, and will be discovered by the ROS2 nodes.
 - each bridge will forward the `ros_discovery_info` data (in a less intensive way than the original publications) to the remote bridges. On reception, the remote bridges will convert the original entities' GIDs into the GIDs of the corresponding replicas, and re-publish on DDS the `ros_discovery_info`. The full ROS graph can then be discovered by the ROS2 nodes on each host.




***
##  GurumNetworks GurumDDS: [Documentation](https://www.omgwiki.org/ddsf/doku.php?id=ddsf:public:guidebook:06_append:05_vendors:gurum)| [Repo](https://github.com/ros2/rmw_gurumdds)
### installation:
How to instal is described [here](https://docs.ros.org/en/rolling/Installation/DDS-Implementations/Working-with-GurumNetworks-GurumDDS.html#).





***

# Tuning considerations
## Ros2 real-time:
* [ROS 2 and Real-time](https://discourse.ros.org/t/ros-2-and-real-time/8796)
* [Proposal for Implementation of Real-time Systems in ROS 2](https://design.ros2.org/articles/realtime_proposal.html)
* [Understanding real-time programming](https://docs.ros.org/en/rolling/Tutorials/Demos/Real-Time-Programming.html)

## DDS tuning general:
* [DDS tuning information (ros2 docs)](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html)
### Quality of service (QoS)
### Message size/ block size
### ipfrag params: _time, _high_thresh, 
