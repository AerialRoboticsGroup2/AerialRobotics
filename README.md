The system integrates:

Flight Control: PX4 (SITL mode)
Middleware: ROS 2
Simulation: Gazebo
Control Strategy: Position setpoint streaming via offboard mode

High-level pipeline:

ROS 2 node publishes trajectory setpoints
PX4 receives commands via microRTPS bridge
Flight controller executes position control
Gazebo simulates vehicle dynamics and environment
