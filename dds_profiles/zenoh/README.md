# Zenoh  [Repo](https://github.com/eclipse-zenoh)
* An overview of the configuration can be found [here](https://github.com/eclipse-zenoh/zenoh-plugin-dds#configuration) and explanation [here](https://zenoh.io/docs/manual/abstractions/)
* An example configuration with comments can be found [DEFAULT_CONFIGURATION.json5](https://github.com/eclipse-zenoh/zenoh-plugin-dds/blob/master/DEFAULT_CONFIG.json5) or look at the local copy.

## Configuration 

`zenoh-bridge-dds` can be configured via a JSON5 file passed via the `-c`argument. You can see a commented example of such configuration file: [`DEFAULT_CONFIG.json5`](DEFAULT_CONFIG.json5).

The `"dds"` part of this same configuration file can also be used in the configuration file for the zenoh router (within its `"plugins"` part). The router will automatically try to load the plugin library (`zenoh-plugin_dds`) at startup and apply its configuration.

`zenoh-bridge-dds` also accepts the following arguments. If set, each argument will override the similar setting from the configuration file:
 * zenoh-related arguments:
   - **`-c, --config <FILE>`** : a config file
   - **`-m, --mode <MODE>`** : The zenoh session mode. Default: `peer` Possible values: `peer` or `client`.  
      See [zenoh documentation](https://zenoh.io/docs/getting-started/key-concepts/#deployment-units) for more details.
   - **`-l, --listen <LOCATOR>`** : A locator on which this router will listen for incoming sessions. Repeat this option to open several listeners. Example of locator: `tcp/localhost:7447`.
   - **`-e, --peer <LOCATOR>`** : A peer locator this router will try to connect to (typically another bridge or a zenoh router). Repeat this option to connect to several peers. Example of locator: `tcp/<ip-address>:7447`.
   - **`--no-multicast-scouting`** : disable the zenoh scouting protocol that allows automatic discovery of zenoh peers and routers.
   - **`-i, --id <hex_string>`** : The identifier (as an hexadecimal string - e.g.: 0A0B23...) that the zenoh bridge must use. **WARNING: this identifier must be unique in the system!** If not set, a random UUIDv4 will be used.
   - **`--group-member-id <ID>`** : The bridges are supervising each other via zenoh liveliness tokens. This option allows to set a custom identifier for the bridge, that will be used the liveliness token key (if not specified, the zenoh UUID is used).
   - **`--rest-http-port <rest-http-port>`** : set the REST API http port (default: 8000)
 * DDS-related arguments:
   - **`-d, --domain <ID>`** : The DDS Domain ID. By default set to `0`, or to `"$ROS_DOMAIN_ID"` is this environment variable is defined.
   - **`--dds-localhost-only`** : If set, the DDS discovery and traffic will occur only on the localhost interface (127.0.0.1).
     By default set to false, unless the "ROS_LOCALHOST_ONLY=1" environment variable is defined.
   - **`-f, --fwd-discovery`** : When set, rather than creating a local route when discovering a local DDS entity, this discovery info is forwarded to the remote plugins/bridges. Those will create the routes, including a replica of the discovered entity. More details [here](#full-support-of-ros-graph-and-topic-lists-via-the-forward-discovery-mode)
   - **`-s, --scope <String>`** : A string used as prefix to scope DDS traffic when mapped to zenoh keys.
   - **`-a, --allow <String>`** :  A regular expression matching the set of 'partition/topic-name' that must be routed via zenoh.
     By default, all partitions and topics are allowed.  
     If both 'allow' and 'deny' are set a partition and/or topic will be allowed if it matches only the 'allow' expression.  
     Repeat this option to configure several topic expressions. These expressions are concatenated with '|'.
     Examples of expressions: 
        - `.*/TopicA` will allow only the `TopicA` to be routed, whatever the partition.
        - `PartitionX/.*` will allow all the topics to be routed, but only on `PartitionX`.
        - `cmd_vel|rosout` will allow only the topics containing `cmd_vel` or `rosout` in their name or partition name to be routed.
   - **`--deny <String>`** :  A regular expression matching the set of 'partition/topic-name' that must NOT be routed via zenoh.
     By default, no partitions and no topics are denied.  
     If both 'allow' and 'deny' are set a partition and/or topic will be allowed if it matches only the 'allow' expression.  
     Repeat this option to configure several topic expressions. These expressions are concatenated with '|'.
   - **`--max-frequency <String>...`** : specifies a maximum frequency of data routing over zenoh per-topic. The string must have the format `"regex=float"` where:
       - `"regex"` is a regular expression matching the set of 'partition/topic-name' for which the data (per DDS instance) must be routedat no higher rate than associated max frequency (same syntax than --allow option).
       - `"float"` is the maximum frequency in Hertz; if publication rate is higher, downsampling will occur when routing.

       (usable multiple times)
   - **`--queries-timeout <Duration>`**: A duration in seconds (default: 5.0 sec) that will be used as a timeout when the bridge
     queries any other remote bridge for discovery information and for historical data for TRANSIENT_LOCAL DDS Readers it serves
     (i.e. if the query to the remote bridge exceed the timeout, some historical samples might be not routed to the Readers,
     but the route will not be blocked forever).
   - **`-w, --generalise-pub <String>`** :  A list of key expressions to use for generalising the declaration of
     the zenoh publications, and thus minimizing the discovery traffic (usable multiple times).
     See [this blog](https://zenoh.io/blog/2021-03-23-discovery/#leveraging-resource-generalisation) for more details.
   - **`-r, --generalise-sub <String>`** :  A list of key expressions to use for generalising the declaration of
     the zenoh subscriptions, and thus minimizing the discovery traffic (usable multiple times).
     See [this blog](https://zenoh.io/blog/2021-03-23-discovery/#leveraging-resource-generalisation) for more details.

