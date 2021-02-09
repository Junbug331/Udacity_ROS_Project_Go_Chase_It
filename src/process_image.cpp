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
		ball_chaser::DriveToTarget srv;
		srv.request.linear_x = lin_x;
		srv.request.angular_z = ang_z;
	
		if (!client.call(srv))
			ROS_ERROR("Failed to call service /ball/chaser/command_bot");
	
	}

  int linearIdx(int row, int col, int colNum)
  {
    return row*colNum + col;
  }

  // This callback function continuously executes and reads the image data
  void process_image_callback(const sensor_msgs::Image img)
  {

    int white_pixel = 255;

    // flag for whether ball is detected in image or not
    bool ball_detected = false;

    // Detected ball's left and right most of ball position(column-wise)
    // for calculation ball's center pos.
    int far_left_ball_pos = img.step-1;
    int far_right_ball_pos = 0;

    // Range for left - forward - right
    int maxLeft = (img.step*0.4) - 1;
    int minRight = img.step - maxLeft - 1;
      
    for (int i=0; i<img.height; i++)
    {
      for (int j=0; j<img.step; j++)
      {
        if (img.data[linearIdx(i,j,img.step)])
        {
          ball_detected = true;
          far_left_ball_pos = std::min(far_left_ball_pos, j);
          far_right_ball_pos = std::max(far_right_ball_pos, j);  
        }
      }
    }
    
    if (ball_detected)
    {
      int ball_center_pos = (far_left_ball_pos + far_right_ball_pos)/2;
      
      // Change its angular position from left to center
      if (ball_center_pos <= maxLeft)
      {
        drive_robot(0.0, 0.2);
      }
      // Change its angular position from right to center
      else if (ball_center_pos >= minRight)
      {
        drive_robot(0.0, -0.2);
      }
      // When ins angular position is in range for center, move forward
      else
      {
        drive_robot(0.3, 0.0);
      }
    }    
    // Stop moving when ball is not detected.
    else
    {
      drive_robot(0.0, 0.0);
    }
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
