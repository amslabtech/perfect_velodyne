//file: ensparse.cpp
//package: perfect_velodyne
//author: shogo shimizu
//last update: 2013.12.20

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl_conversions/pcl_conversions.h>


#include <Eigen/Core>
#include <Eigen/SVD>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <velodyne_msgs/VelodyneScan.h>


using namespace std;
using namespace Eigen;


sensor_msgs::PointCloud2 cloud_ros;
pcl::PointCloud<pcl::PointXYZI> cloud_pcl;
ros::Publisher pub;



void pc_callback(sensor_msgs::PointCloud2::Ptr msg)
{
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*msg, *cloud);
	for(int i=0;i<(int)cloud->points.size();i++){
		if( (i%9) == 0){
			pcl::PointXYZI p;
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;
			cloud_pcl.points.push_back(p);
		}
	}
	pcl::toROSMsg(cloud_pcl, cloud_ros);
	cloud_pcl.points.clear();

	cloud_ros.header.frame_id = "/velodyne";
	cloud_ros.header.stamp = ros::Time::now();
	pub.publish(cloud_ros);

}

int main (int argc, char** argv)
{
	cout << "start" << endl;
	ros::init(argc, argv, "Ensparse");
  	ros::NodeHandle n;
	ros::Rate roop(2);
	ros::Subscriber sub = n.subscribe("/velodyne_points",1,pc_callback);
	pub = n.advertise<sensor_msgs::PointCloud2>("/ensparsed_points",1);
	ros::spin();
	
	return 0;
}
