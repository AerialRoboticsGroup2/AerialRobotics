The system integrates:

1.Flight Control: PX4 (SITL mode)

2.Middleware: ROS 2

3.Simulation: Gazebo

4.Control Strategy: Position setpoint streaming via offboard mode

High-level pipeline:

1.ROS 2 node publishes trajectory setpoints

2.PX4 receives commands via microRTPS bridge

3.Flight controller executes position control

4.Gazebo simulates vehicle dynamics and environment

