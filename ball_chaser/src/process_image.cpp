#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
	
    // Call the safe_move service and pass the drive commands
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
	// Camera is R8G8B8 encoded which means 24-bit pixel format, 8 bits for red, 8 bits green and 8 bits blue.
	// There is 3 bytes per pixles. So the number of bytes across the width of pitcture is 3xwidth=step 
	// img.step is the row length in bytes
	// img.data[]  is the actual matrix data, size is (step * rows)
	// img.height is the image height, that is, number of rows  
	// A pixel is pure white is red=255, green=255 and blue=255 
 
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	int white_pixel_cnt=0, collumn_acc=0, ball_avg_pos=0;
	
	for(int row_cnt=0; row_cnt<img.height; row_cnt++){ 			     // Step through the rows one by one
		for(int column_cnt=0; column_cnt<img.step; column_cnt+=3){   // Step through bytes for red, green and blue 
			if((img.data[column_cnt + row_cnt*img.step]==255) && 	 // Check red
			   (img.data[column_cnt + row_cnt*img.step + 1]==255) && // Check green
			   (img.data[column_cnt + row_cnt*img.step + 2]==255)){  // Check blue
				   white_pixel_cnt++;            					 // White pixel found
				   collumn_acc = collumn_acc + column_cnt/3;		 	 // Accumulate horizontal position
			}
		}
	}
	
	if (white_pixel_cnt != 0){  
		ball_avg_pos = collumn_acc / white_pixel_cnt; // Average horizontal position, center of the ball, is the accumilated positions divieded by the number of white pixels
		if (ball_avg_pos<(img.width/3)){ // White pixels on left 1/3rd of image -> Go right
		  drive_robot(0.0, 0.5);
		}
		else if (ball_avg_pos>(img.width*2)/3){ // White pixels on right 1/3rd of image -> Go left
	      drive_robot(0.0, -0.5);
		}
		else{ // White pixels in the center 1/3rd of image -> Go straight
		  drive_robot(0.5, 0.0);
		}			
	}
	else{
		drive_robot(0.0, 0.0); // No white pixels -> Stop
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
