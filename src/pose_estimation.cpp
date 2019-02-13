//pose_estimationcpp
//s.shimizu
//create 2013.12.07
//update 2013.12.07 

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point32.h>
#include <velodyne_msgs/VelodyneScan.h>

//#include </home/amsl/my_ros_pkg/pcl_test/include/pcl_test/clustering.h>
#include <perfect_velodyne/clustering.h>

//#include </home/amsl/my_ros_pkg/perfect_velodyne/include/perfect_velodyne/rtime_registration.h>
#include <perfect_velodyne/rtime_registration.h>


using namespace std;
using namespace Eigen;

const char* file_name = "/home/amsl/my_ros_pkg/share_area/fb.csv";
//const char* file_name = "fb.csv";

sensor_msgs::PointCloud2 pc;
sensor_msgs::Imu pose;
visualization_msgs::Marker marker;
ros::Publisher pub;
ros::Publisher pub_pose;
ros::Publisher pub_pair;
Matrix3d state = Matrix3d::Identity();
Registration reg;

void joy_callback(sensor_msgs::JoyConstPtr msg){
	if(msg->buttons[0]==1)
		reg.reset();
}
/*
void pose_callback(sensor_msgs::ImuConstPtr msg){
	cout << "カルマンフィルタからのフィードバック" << endl;
	Matrix3d rot;
	double y = -M_PI / 180.0 * msg->orientation.z;
	rot << cos(y), -sin(y), 0.0, sin(y), cos(y), 0.0, 0.0, 0.0, 1.0;
	reg.specialFeedBack(rot);
}
*/
void pc_callback(sensor_msgs::PointCloud2ConstPtr msg){

	double y = 0.0;
	FILE* fp;
	if( (fp = fopen(file_name,"r")) ==NULL){
		cout << "can't find fb.csv. " << endl;
		cout << "stop this program..." << endl;
		exit(1);
	}else{
		fscanf(fp,"%lf",&y);
		fclose(fp);
	}
	y *= M_PI/180.0;
	Matrix3d rot;
	rot << cos(y), -sin(y), 0.0, sin(y), cos(y), 0.0, 0.0, 0.0, 1.0;
	reg.specialFeedBack(rot);

//	cout << "callback" << endl;
	reg.setQuery(msg);
	reg.rotate();
	reg.pairing();
	///////////////////
	reg.rotate_inv();
	reg.getPair(marker);
	reg.rotate();
	//////////////////
	reg.process();
	reg.check();
	reg.output(pc);
	//pc.header.frame_id = "/velodyne";
	pc.header.stamp = ros::Time::now();

	marker.header.frame_id = "/velodyne";
	marker.header.stamp = ros::Time::now();

	reg.getPose(pose);
	pose.header.frame_id = "velodyne";
	pose.header.stamp = ros::Time::now();

	pub.publish(pc);
	pub_pose.publish(pose);
	pub_pair.publish(marker);
	cout << endl;
//	cout << "*************************************" << endl;
}

int main (int argc, char** argv)
{
	cout << "Azimuth Estimation Start" << endl;
	ros::init(argc, argv, "AzimuthEstimation");
  	ros::NodeHandle n;
	//ros::Rate roop(2);
	ros::Subscriber sub = n.subscribe("/perfect_velodyne/principal_components",1,pc_callback);
	ros::Subscriber sub_joy = n.subscribe("/joy",1,joy_callback);
//	ros::Subscriber sub_pose = n.subscribe("/perfect_velodyne/filtered_pose",1,pose_callback);
	pub = n.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/tranfered_principal_components",1);
	pub_pose = n.advertise<sensor_msgs::Imu>("/perfect_velodyne/pose",1);
	pub_pair = n.advertise<visualization_msgs::Marker>("/perfect_velodyne/pair",1);
	//while(ros::ok()){	
	ros::spin();
	//	roop.sleep();
	//}
	return 0;
}
