#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chase/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {
	
	ROS_INFO("DriveToTarget request received");

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = 0.5;
    motor_command.angular.z = 0.0;
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    res.msg_feedback = "Velocities were set to: linear_x: " + to_string((double)motor_command.linear.x)
		 + ", angular_z: " + to_string((double)motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer server = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ros::spin();

    return 0;
}