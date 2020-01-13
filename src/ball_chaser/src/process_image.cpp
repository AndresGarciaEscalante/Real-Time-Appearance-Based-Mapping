#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{ 
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    // Call the command_robot service and pass the requested linear and angular velocities
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    float pos = 0.0;
    float linear_vel_x, angular_vel_z;
    bool ball_present = false;
    // TO-DO: Missing the orientation of the robot to go left, right, or forwards.
    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < img.height * img.step; i=i+3) {
        pos = i % img.width;
        if(img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255){ // Cheking if the RGB values of each pixel
            ball_present = true;
            break;
        }  
    }
    //Ball present
    if (ball_present == true){
        if(pos < 266){ // Go_left
            ROS_INFO_STREAM("Robot Going Left");
            linear_vel_x = 0.0;
            angular_vel_z = 0.5;
            drive_robot(linear_vel_x,angular_vel_z);
        }
        else if(pos >= 266 && pos < 533){ // Go_fordwards
            ROS_INFO_STREAM("Robot Going fordward");
            linear_vel_x = 0.5;
            angular_vel_z = 0.0;
            drive_robot(linear_vel_x,angular_vel_z);
        }

        else { // Go_right
            ROS_INFO_STREAM("Robot Going right");
            linear_vel_x = 0.0;
            angular_vel_z = -0.5;
            drive_robot(linear_vel_x,angular_vel_z);
        }
    }
    
    // The ball is not present then stop the robot
    else{
        ROS_INFO_STREAM("Robot stopped");
        linear_vel_x = 0.0;
        angular_vel_z = 0.0;
        drive_robot(linear_vel_x,angular_vel_z);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
