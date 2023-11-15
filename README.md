# gazebo\_ros\_omni\_drive

----------

*A omnidirectional (four mechanum wheels ) drive plugin for gazebo. Based on the diffdrive plugin and planar move plugin.

*The planar move used includes an interesting correction made by F.M. Rico <https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1504/commits> in 2023

*prof.: Miguel Hernando (2023)*
### LIB REQUIREMENTS:

    sudo apt install ros-foxy-gazebo-dev 
    sudo apt install ros-foxy-gazebo-msgs 
    sudo apt install ros-foxy-gazebo-ros


<https://github.com/mhernando/gz_rosa_control>

 
### PLUGIN PARAMETERS:

![Image of rubik](images/gz_ros_plugin_parameters.jpg)

The figure shows the main parameters needed for the similation of omnidirectional drive and also de used conventions.

- commandTopic: 
- odometryTopic:
- odometryFrame:
- odometryRate:
- robotBaseFrame:
- wheel_radius:
- base_lenght:
- base_width:
- front_left_joint:
- rear_left_joint:
- front_right_wheel:
- rear_right_wheel:
- wheel_max_speed:
- wheel_acceleration:
- joint_config:
 

###example of use in a urdf file:

```
<gazebo>
    <plugin name="rosa_controller" filename="libgazebo_ros_omni_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometryRate>20.0</odometryRate>
      <robotBaseFrame>base_footprint</robotBaseFrame>
      <wheel_radius>0.075</wheel_radius>
      <base_length>0.65</base_length>
      <base_width>0.65</base_width>
      <front_left_joint>wheel_front_left_joint</front_left_joint>
      <front_right_joint>wheel_front_right_joint</front_right_joint>
      <rear_left_joint>wheel_back_left_joint</rear_left_joint>
      <rear_right_joint>wheel_back_right_joint</rear_right_joint>
      <wheel_max_speed> 20.0 </wheel_max_speed>
      <wheel_acceleration> 10.0</wheel_acceleration>
      <joint_config>1 1 -1 -1</joint_config>
    </plugin>
  </gazebo>
```