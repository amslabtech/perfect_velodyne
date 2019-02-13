//principal_components_extractor.cpp
//s.shogo
//create 2013.10.31
//update 2013.11.03 

#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <velodyne_msgs/VelodyneScan.h>
//#include </home/amsl/my_ros_pkg/pcl_test/include/pcl_test/clustering.h>
#include <perfect_velodyne/clustering.h>

#define N_pub 10

using namespace std;
using namespace Eigen;


sensor_msgs::PointCloud2 pc;
ros::Publisher pub;
ros::Publisher master_pub[N_pub];
int min_n = 0;
vector<sensor_msgs::PointCloud2> master_pc;

void pc_callback(sensor_msgs::PointCloud2ConstPtr msg){
	cout << "callback" << endl;
	Clustering clustering;
	clustering.setMinMaxNum(min_n,INT_MAX);
	//clustering.setThreshold(0.6, 0.1);	//angle and distance
	clustering.setThreshold(0.80, 0.1);	//angle and distance
	clustering.putTank(msg);
	clustering.process();
	clustering.calcWeight();
	clustering.showClusterSize();
	clustering.showClusters();
	clustering.getClusters(pc);
	int n_cluster = 0;
	clustering.getInfo(&master_pc,n_cluster);
	pc.header.frame_id = "/velodyne";	//principal_components
	pc.header.stamp = ros::Time::now();	//principal_components
	pub.publish(pc);
	for(size_t i=0;(i<(size_t)n_cluster)&&(i<N_pub);i++){
		master_pc[i].header.frame_id = "/velodyne";
		master_pc[i].header.stamp = ros::Time::now();
		master_pub[i].publish(master_pc[i]);
	}

}

int main (int argc, char** argv)
{
	//cout << "min = " ; cin >> min_n;
	min_n = 100;
	cout << "PrincipalComponentsExtractor start" << endl;
	ros::init(argc, argv, "PrincipalComponentsExtractor");
  	ros::NodeHandle n;
	ros::Rate roop(2);
	ros::Subscriber sub = n.subscribe("/perfect_velodyne/normal_sphere",1,pc_callback);
	pub = n.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/principal_components",1);
	for(int i=0;i<N_pub;i++){
		char tpc[512];
		sprintf(tpc,"/perfect_velodyne/pc/component%d",i);
		master_pub[i] = n.advertise<sensor_msgs::PointCloud2>(tpc,1);
	}
	//while(ros::ok()){	
		ros::spin();
	//	roop.sleep();
	//}
	return 0;
}
