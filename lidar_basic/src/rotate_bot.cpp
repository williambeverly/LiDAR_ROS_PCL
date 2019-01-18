#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "lidar_basic/AdjustAngularVel.h"

class RobotCommandService
{
public:
	RobotCommandService()
	{
		this->publisher = this->nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		this->service = this->nh.advertiseService("/lidar_basic/command_robot", &RobotCommandService::serviceCallback, this);
		this->tolerance = 0.02;
	}

	bool 
	serviceCallback(lidar_basic::AdjustAngularVel::Request& req, lidar_basic::AdjustAngularVel::Response& res)
	{
		//ROS_INFO("Handle angular velocity request has been called");

		// create a new message
		geometry_msgs::Twist service_request;
		
		// set the rotate robot value
		if (fabs(req.angular_z) > this->tolerance)
			service_request.angular.z = req.angular_z;
		else
			service_request.angular.z = 0.0;

		this->publisher.publish(service_request);

		// return a message for the feedback
		res.msg_feedback = "Requested z: " + std::to_string(req.angular_z);
		ROS_INFO_STREAM(res.msg_feedback);
		return true;
	}

private:
	ros::NodeHandle nh;
	ros::Publisher publisher;
	ros::ServiceServer service;
	double tolerance;
};

int main(int argc, char** argv)
{
	// initialise the ROS node
	ros::init(argc, argv, "rotate_bot");

	std::cout << "Rotate_bot node initialised" << std::endl;
	
	// create an instance of the class
	RobotCommandService rotate;

	// handle ROS communicate events
	ros::spin();

	return 0;
}


