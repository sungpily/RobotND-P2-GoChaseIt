#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands
ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, 
            ball_chaser::DriveToTarget::Response& res) {

    ROS_INFO("DriveToTarget received- linear_x:%1.2f, angular_z:%1.2f",
                (float)req.linear_x, (float)req.angular_z);

    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    motor_command_publisher.publish(motor_command);

    return true;
}

int main(int argc, char** argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishering a messae of type
    // geometry_msgs::Twist on the robot actuation topic with 
    // a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a 
    // handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ros::spin();

    return 0;
}
