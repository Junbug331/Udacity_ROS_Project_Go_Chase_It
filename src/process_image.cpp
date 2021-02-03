#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage
{
public:
	ProcessImage()
	{
		// Define a client service capable of requesting services from command_robot
		client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

		// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
		sub1 = n.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
	}
	// This function calls the command_robot service to stop the robot
	void stop_robot()
	{
		ball_chaser::DriveToTarget srv;
		srv.request.linear_x = 0.0;
		srv.request.angular_z = 0.0;

		if (!client.call(srv))
			ROS_ERROR("Failed to call service /ball/chaser/command_bot");
	}

	// This function calls the command_robot service to drive the robot in the specified direction
	void drive_robot(float lin_x, float ang_z)
	{
    	// TODO: Request a service and pass the velocities to it to drive the robot
		ball_chaser::DriveToTarget srv;
		srv.request.linear_x = lin_x;
		srv.request.angular_z = ang_z
	
		if (!client.call(srv))
			ROS_ERROR("Failed to call service /ball/chaser/command_bot");
	
	}

	// This callback function continuously executes and reads the image data
	void process_image_callback(const sensor_msgs::Image img)
	{

	    int white_pixel = 255;

	    // TODO: Loop through each pixel in the image and check if there's a bright white one
	    // Then, identify if this pixel falls in the left, mid, or right side of the image
    	// Depending on the white ball position, call the drive_bot function and pass velocities to it
    	// Request a stop when there's no white ball seen by the camera
	}

private:
    ros::NodeHandle n;
	ros::ServiceClient client;
	ros::Subscriber sub1;
};


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
	
	ProcessImage processImage;

    // Handle ROS communication events
    ros::spin();

    return 0;
}
