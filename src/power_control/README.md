# **Power Control for ROS**

This package is used to control the power state of machines that are running ROS. This package uses service calls to poweroff or reboot one or multiple systems.

## Basic concept
This package can be control the power state of multiple systems. There will be one master system that will communicate with slave systems whose power state depends on the master. For example, if the master system is asked to be rebooted, the master will execute a reboot service call to all the configured slave systems. 

If a slave system is asked to be rebooted, the master system is unaffected as the slave cannot make a service call to the master to turn of the systems.

## Setup

For power control to work properly, the current user of the sytem needs to be be given passwordless `sudo` permission for `reboot`, `poweroff` and `shutdown` is required. For this, the following setup script needs to be run. By default, the passwordless permission is given to the current user that is executing the script.

```bash
# For current user (Recommended)
rosrun power_control setup_pwr_ctrl_permission.sh

# For different user
rosrun power_control setup_pwr_ctrl_permission.sh user_name 
```
## Usage

### Setting up the configurations

The package must be configured properly by setting the `system_name` with the device name and also setting the `is_system_master` parameters as well. If the system is set to master, the `slave_system_names` parameters will be used to find the slave system names. The services required controlling the power of the systems are automatically generated from these names that are set in the configuration file.

### Usage within the launch file

The launch file can be included in the main launch file of the system as given below. In this example, a different configuration YAML file is given rather than using the default configuration file included with the package. It is recommended to provided a different configuration file rather than modifying the default configuration file provided in the package.

```xml
<launch>

    <!-- Other launch files -->

    <include file="$(find power_control)/launch/power_control.launch">
        <arg name="system_name" value="device_name"> <!--Node name will be set to power_control_device_name -->
        <arg name="config_file" value="/path/to/config_file.yaml"> <!-- This is recomended -->
        <arg name="output_to" value="screen">
    </include>

</launch>
```

### Services

Once the file containing the power control is launched, master and slave systems' power control services will be available. By calling the master's power control services, the slave systems power control is also called as mentioned above.

**NOTE: The separate master and slave systems should be networked such that the power control nodes are able to find each other. This can be done by setting ROS_MASTER_URI and ROS_IP environment variables**

```bash
# For master system poweroff
rosservice call /power_control/master/master_device_name/poweroff

# For master system reboot
rosservice call /power_control/master/master_device_name/reboot

# For slave system(s) poweroff
rosservice call /power_control/slave/slave_device_name/poweroff

# For slave system(s) reboot
rosservice call /power_control/slave/slave_device_name/reboot
```

### Testing
For testing purposes or if the master or slave should not be powered off or rebooted, the master's or slave's power control can be dynamically enabled or disabled using `rqt_reconfigure`. First, run `rqt_reconfigure` and find the power control node for the required system. This is usually `power_control_device_name`. Then uncheck or check the tick box to enable or disable power control for that system.