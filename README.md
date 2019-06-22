# ROS Workspace

ROS programming exercise showing examples on how to use Publisher, Subscriber, ServiceServer and ServiceClient

## Package "my_robot"

defines environment and robot

## Package "ball_chaser"

implements a node "drive_bot" that creates a server for "/ball_chaser/command_robot" service and another node "process_image" that creates a client to process camera images and call the service to drive the robot
