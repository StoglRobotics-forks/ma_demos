# Cyclon DDS [REPO](https://github.com/eclipse-cyclonedds/cyclonedds#run-time-configuration)
* [Documentation](https://cyclonedds.io/docs/cyclonedds/latest/index.html)
* [List of available settings](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/options.md)
* [Configuration guide](https://cyclonedds.io/docs/cyclonedds/latest/config/index.html)



## Short how to
1. create a `.xml` file e.g. `example.xml`
2. set configuration as wanted in the `.xml` file
3. Make the configuration visible: `export CYCLONEDDS_URI=file://$PWD/example.xml`

## Explanation of the [`example.xml`](https://github.com/StoglRobotics-forks/ma_demos/tree/main/dds_profiles/cyclon_dds/example.xml)
* `Interfaces` can be used to override the interfaces selected by default.
  Members are
  * `NetworkInterface[@autodetermine]` tells Cyclone DDS to autoselect the interface it deems best.
  * `NetworkInterface[@name]` specifies the name of an interface to select (not shown above, alternative for autodetermine).
  * `NetworkInterface[@address]` specifies the ipv4/ipv6 address of an interface to select (not shown above, alternative for autodetermine).
  * `NetworkInterface[@multicast]` specifies whether multicast should be used on this interface.
    The default value 'default' means Cyclone DDS will check the OS reported flags of the interface and enable multicast if it is supported.
    Use 'true' to ignore what the OS reports and enable it anyway and 'false' to always disable multicast on this interface.
  * `NetworkInterface[@priority]` specifies the priority of an interface.
    The default value (`default`) means priority `0` for normal interfaces and `2` for loopback interfaces.
* `AllowMulticast` configures the circumstances under which multicast will be used.
  If the selected interface doesn't support it, it obviously won't be used (`false`); but if it does support it, the type of the network adapter determines the default value.
  For a wired network, it will use multicast for initial discovery as well as for data when there are multiple peers that the data needs to go to (`true`).
  On a WiFi network it will use it only for initial discovery (`spdp`), because multicast on WiFi is very unreliable.
* `EnableTopicDiscoveryEndpoints` turns on topic discovery (assuming it is enabled at compile time), it is disabled by default because it isn't used in many system and comes with a significant amount of overhead in discovery traffic.
* `Verbosity` allows control over the tracing, "config" dumps the configuration to the trace output (which defaults to "cyclonedds.log", but here the process id is appended).
  Which interface is used, what multicast settings are used, etc., is all in the trace.
  Setting the verbosity to "finest" gives way more output on the inner workings, and there are various other levels as well.
* `MaxMessageSize` controls the maximum size of the RTPS messages (basically the size of the UDP payload).
  Large values such as these typically improve performance over the (current) default values on a loopback interface.
* `WhcHigh` determines when the sender will wait for acknowledgements from the readers because it has buffered too much unacknowledged data.
  There is some auto-tuning, the (current) default value is a bit small to get really high throughput.