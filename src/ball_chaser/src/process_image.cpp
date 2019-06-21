#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the 
// specified direction
void drive_robot(float lin_x, float ang_z) {

    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if (!client.call(srv))
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img) {

    int white_pixel = 255;
    bool should_drive = false;
    int coord = 0;

    // Scan the image and see whether the ball is in the field of view
    for (int i = 0; i < img.height * img.step; i++) {

        // Check the start of RGB
        if (i % 3 == 0) {  

            // Check if img.data[i] == img.data[i+1] == img.data[i+2] == 255
            if (img.data[i] + img.data[i+1] + img.data[i+2] == white_pixel * 3) {
                should_drive = true;
                coord = i % img.step;
                break;
            }
        }
    }
    
    if (should_drive) {
        // Assign linear velocity
        float lin_x = 0.1;

        // Compute angular velocity
        float ang_z = -(coord - 1200) / 1200. * 3.14 / 4;

        drive_robot(lin_x, ang_z);
    } else {
        drive_robot(0, 0);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data 
    // inside the process_image_callback function
    ros::Subscriber subl = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
