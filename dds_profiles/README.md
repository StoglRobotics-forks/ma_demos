# DDS in ROS2
Overview of ros2 middlewares: (Rolling default is Fast DDS)

| **Product name**          | **License**                   | **RMW implementation**    |**Status** | **Working?** |
| :-------------------:     | :----------:                  | :----------------------: | :-------------: | :---: |
| [eProsima **Fast DDS**](https://fast-dds.docs.eprosima.com/en/latest/)         | Apache 2                      | ``RMW_IMPLEMENTATION=rmw_fastrtps_cpp``      | Full support. **Default RMW**. Packaged with binary releases.| YES. out of the box|
| [Eclipse **Cyclone DDS**](https://projects.eclipse.org/projects/iot.cyclonedds)       | Eclipse Public License v2.0   | ``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp``      | Full support. Packaged with binary releases. | YES. binary install |
| [RTI **Connext DDS**](https://www.rti.com/products)           | commercial, research          | ``RMW_IMPLEMENTATION=rmw_connextdds``      | Full support. Support included in binaries, but Connext installed separately. | YES. Need to install correct binary and license |
|[Eclipse **Zenoh**](https://zenoh.io/)  |?|?|?|?|
||||||
| [GurumNetworks **GurumDDS**](https://gurum.cc/index_eng)    | commercial                    |  Not working      | Community support. Support included in binaries, but GurumDDS installed separately. |NO. commercial license needed|

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
##  GurumNetworks GurumDDS: [Documentation](https://www.omgwiki.org/ddsf/doku.php?id=ddsf:public:guidebook:06_append:05_vendors:gurum)| [Repo](https://github.com/ros2/rmw_gurumdds)
### installation:
How to instal is described [here](https://docs.ros.org/en/rolling/Installation/DDS-Implementations/Working-with-GurumNetworks-GurumDDS.html#).




***
##  Eclipse Zenoh: [Documentation](https://zenoh.io/docs/getting-started/first-app/) | [Repo](https://github.com/eclipse-zenoh)
### installation:
How to instal is described [here](https://zenoh.io/blog/2021-04-28-ros2-integration/).


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
