# FAST DDS [REPO](https://github.com/eProsima/Fast-DDS)
* [How to set xml profiles](https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html#xml-profiles)
* [Example how to set tcp as default](https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/)

## [Short how to](https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html#env-vars-fastrtps-default-profiles-file)
1. create a `.xml` file e.g. `example.xml`
2. set configuration as wanted in the `.xml` file
3. Make the configuration visible: `export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/example.xml`