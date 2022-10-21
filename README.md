# a1_driver

This package is in charge of publishing all the data coming from the A1 robot sensors and control the velocity of the robot in the XY axis.

## Publishers

- /a1_state ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

- /feet_forces ([unitree_legged_msgs/FeetForces](https://github.com/CardiffUniversityComputationalRobotics/unitree_ros_to_real/blob/main/unitree_legged_msgs/msg/FeetForces.msg))

  Force in the Z axis for each feet of the robot.

- /feet_velocities ([unitree_legged_msgs/FeetVelocities](https://github.com/CardiffUniversityComputationalRobotics/unitree_ros_to_real/blob/main/unitree_legged_msgs/msg/FeetVelocities.msg))

  Velocities of the end of the feet of the robot in the XYZ axis.

- /feet_polygon ([geometry_msgs/PolygonStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PolygonStamped.html))

  Position of the foot of the robot in XYZ axis.

- /imu_raw ([sensors_msg/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html))

- /pose ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))

  Position of the robot in XYZ. This position is measured by the robot, when the robot does not move, the XY positions are reseted.

- /current_vel ([geometry_msgs/TwistStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistStamped.html))

  Current velocity measured by the robot.

## Subscribers

- /cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
