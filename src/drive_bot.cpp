#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveBot
{
public:
	DriveBot()
	{
		motor_command_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		service = nh.advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
	}

	bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget:: Response &res)
	{
		float lin_x = req.linear_x;
		float ang_z = req.angular_z;
	
		geometry_msgs::Twist motor_command;
		motor_command.linear.x = lin_x;
		motor_command.angular.z = ang_z;
	
		motor_command_publisher.publish(motor_command);

		res.msg_feedback = "requested wheel velocities: " + std::to_string(lin_x) + " wheel angle: " + std::to_string(ang_z);
		ROS_INFO_STREAM(res.msg_feedback);

		return true;		
	}

private:
	ros::NodeHandle nh;
	// ROS::Publisher motor commands;
	ros::Publisher motor_command_publisher;
	ros::ServiceServer service;
};



int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

	DriveBot driveBot;	

    ros::spin();

    return 0;
}
