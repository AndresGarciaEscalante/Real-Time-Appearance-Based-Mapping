#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h" 

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// This function publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{
    // Twist message instance 
    geometry_msgs::Twist motor_command;
    // Assign the values to the Twist type from the request message
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publish linear and angles to drive the robot
    motor_command_publisher.publish(motor_command);
    
    // Message response to the client
    res.msg_feedback = "Linear velocity : " + std::to_string(motor_command.linear.x) + " , angular velocity : " + std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot",handle_drive_request);
    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
    ROS_INFO("Ready to send linear and angular velocities commands");

    // Handle ROS communication events
    ros::spin();
    
    return 0;
}