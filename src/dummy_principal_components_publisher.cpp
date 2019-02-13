//dummy_principal_components_publisher.cpp
//s.shimizu
//2013.11.4

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <Eigen/Core>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;
using namespace Eigen;

sensor_msgs::PointCloud2 pc;
pcl::PointCloud<pcl::PointXYZI> cloud;
ros::Publisher pub;
Matrix3d state = Matrix3d::Identity();
MatrixXd org(3,6);

Matrix3d rot = Matrix3d::Identity();

double g_noise = 0.0;

double gaussRand(double sigma){
	double x = (double)rand()/(RAND_MAX + 1.0);
	double y = (double)rand()/(RAND_MAX + 1.0);
	double rsl = sigma * sqrt(-2*log(x))*cos(2*M_PI*y);
	return rsl;
}

void addNoise(MatrixXd &m){
	for(int i=0;i<m.cols();i++)
	for(int j=0;j<m.rows();j++)
		m(j,i) += gaussRand(g_noise);
/*	
	for(int i=0;i<m.cols();i++)
		m.col(i).normalize();
*/
}

void joy_callback(sensor_msgs::JoyConstPtr msg){
	cout << "joy" << endl;
	rot = Matrix3d::Identity();
	double rol,pit,yaw;
	yaw = msg->axes[0]*1.5*M_PI;
	pit = msg->axes[1]*1.5*M_PI;
	rol = msg->axes[2]*1.5*M_PI;
	g_noise = (msg->axes[3]+1.0)*0.1;
	Matrix3d tmp;
	tmp << 1,0,0, 0,cos(rol),-sin(rol), 0,sin(rol),cos(rol);
	rot = tmp * rot;
	tmp << cos(pit),0,-sin(pit), 0,1,0, sin(pit),0,cos(pit);
	rot = tmp* rot;
	tmp << cos(yaw),-sin(yaw),0, sin(yaw),cos(yaw),0, 0,0,1;
	rot = tmp * rot;
	cout << "rot = " << endl << rot << endl;
}

void put (MatrixXd m, MatrixXd n) {
	cloud.points.resize(m.cols());
	for(size_t i=0;i<cloud.points.size();i++){
		Vector3d tmp;
		tmp << m(0,i)+n(0,i), m(1,i)+n(1,i), m(2,i)+n(2,i);
		tmp.normalize();

		cloud.points[i].x = tmp(0);
		cloud.points[i].y = tmp(1);
		cloud.points[i].z = tmp(2);
		
		//double noise_level = (m.col(i).cross(tmp.row(i))).norm();
		Vector3d a1,a2,a3;
		a1 = m.col(i);
		a2 = tmp;
		a3 = a1.cross(a2);
		double noise_level = a3.norm();
		cloud.points[i].intensity = 1.0/(noise_level+0.01);
	}
	toROSMsg(cloud, pc);
	pc.header.frame_id = "/velodyne";
	pc.header.stamp = ros::Time::now();
}

int main (int argc, char** argv)
{
	cout << "Dummy Start" << endl;
	ros::init(argc, argv, "Dummy");
  	ros::NodeHandle n;
	ros::Rate roop(10);
	ros::Subscriber sub = n.subscribe("/joy",1,joy_callback);
	pub = n.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/principal_components",1);
	float t = 0.1;
	Matrix3d dif;
	org << 1,0,0,-1,0,0, 0,1,0,0,-1,0, 0,0,1,0,0,-1;
	//org << 0,0,-1,0,0, 1,0,0,-1,0, 0,1,0,0,-1;
	//org << 0,0,0, 1,-1,0, 0,0,-1;
	/*dif <<  cos(t),  sin(t), 0.0,
		-sin(t), cos(t), 0.0,
		0.0,     0.0,    1.0;
	*/
	while(ros::ok()){
		//org = dif * org;
		MatrixXd snd = rot * org;
		MatrixXd noise = MatrixXd::Zero(org.rows(),org.cols());
		addNoise(noise);
		put(snd,noise);
		pub.publish(pc);
		ros::spinOnce();
		roop.sleep();
	}
	return 0;
}

