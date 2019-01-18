#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include "lidar_basic/AdjustAngularVel.h"

class SubscribeProcessPublish
{
public:
    SubscribeProcessPublish()
    {
        // assign subscriber
		this->subscriber = this->nh.subscribe<pcl::PCLPointCloud2>("/velodyne_points", 5, &SubscribeProcessPublish::processLidarMeasurementCallBack, this);
        
		// assign publisher
        this->publisher = this->nh.advertise<pcl::PCLPointCloud2> ("output", 1);

		// create service client		
		this->client = this->nh.serviceClient<lidar_basic::AdjustAngularVel>("/lidar_basic/command_robot");
    }

	void
	rotate_robot(double ang_z)
	{
		lidar_basic::AdjustAngularVel srv;
		srv.request.angular_z = ang_z;

		if(!this->client.call(srv))
			ROS_ERROR("Failed to call service AdjustAngularVel");
	}
    
	void 
    processLidarMeasurementCallBack(const pcl::PCLPointCloud2ConstPtr& cloud)
    {      
		//std::cout << "Received lidar measurement made at " << cloud->header.seq << std::endl;

		// define a new container for the data
		pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
		
		// define a voxelgrid
		pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
		// set input to cloud
		voxelGrid.setInputCloud(cloud);
		// set the leaf size (x, y, z)
		voxelGrid.setLeafSize(0.1, 0.1, 0.1);
		// apply the filter to dereferenced cloudVoxel
		voxelGrid.filter(*cloudVoxel);

		//this->publisher.publish (*cloudVoxel);

		// cascade the floor removal filter and define a container for floorRemoved	
		pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());
		
		// define a PassThrough
		pcl::PassThrough<pcl::PCLPointCloud2> pass;
		// set input to cloudVoxel
		pass.setInputCloud(cloudVoxel);
		// filter along z-axis
		pass.setFilterFieldName("z");
		// set z-limits
		pass.setFilterLimits(-0.2, 1.0);
		pass.filter(*floorRemoved);

        // Publish the data for visualisation
        this->publisher.publish (*floorRemoved);

		// define a point cloud of type PointXYZ
		pcl::PointCloud<pcl::PointXYZ> pclXYZ;

		// copy the contents of the floorRemoved to pclXYZ
		pcl::fromPCLPointCloud2(*floorRemoved, pclXYZ);
		pcl::PointXYZ centroid;
				
		// if 1 or more points were utilised, consider the centroid to be valid
		if(pcl::computeCentroid(pclXYZ, centroid) > 0)
		{
			// define orientation which will be sent to robot
			double orientation = atan2 (centroid.y, centroid.x);
			//std::cout << "Centroid x: " << centroid.x << " y: " << centroid.y << " z: " << centroid.z << " with orientation " << orientation << std::endl;
			this->rotate_robot(orientation);
		}
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
  	ros::ServiceClient client;
  	double tolerance;
};

int main(int argv, char** argc)
{
	// initialise the node	
	ros::init(argv, argc, "process_lidar");

	std::cout << "Process_lidar node initialised" << std::endl;

	// create instance of PublishSubscribe
	SubscribeProcessPublish process;

	// Handle ROS communication events
	ros::spin();

	return 0;
}
