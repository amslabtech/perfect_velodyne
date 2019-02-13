//file: normal_estimation.cpp
//package: perfect_velodyne
//author: shogo shimizu
//last update: 2013.08.03
//2013.11.11	推定された法線を正規化した。

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
#include <omp.h>

#define HNN 4
#define VNN 1
#define VN 32
//#define THRESH_D 0.05 // not bekizyou
#define THRESH_D 0.2

using namespace std;
using namespace Eigen;

sensor_msgs::PointCloud2 ros_pc_c;
sensor_msgs::PointCloud2 ros_pc_n;
sensor_msgs::PointCloud2 ros_pc_nd;
//sensor_msgs::PointCloud pc;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher pub;
ros::Publisher pub_2;
ros::Publisher pub_3;
ros::Publisher pub_4;

const size_t skip = 3;
const float VR = 0.839;

size_t itr(const size_t &num)
{
	size_t tmp;
	size_t tmp_1 = num % VN;

	switch (tmp_1){
		case 0: tmp = 0; break;
		case 1: tmp = 2; break;
		case 2: tmp = 4; break;
		case 3: tmp = 6; break;
		case 4: tmp = 8; break;
		case 5: tmp =  10; break;
		case 6: tmp =  12; break;
		case 7: tmp =  14; break;
		case 8: tmp =  16; break;
		case 9: tmp =  18; break;
		case 10: tmp =  20; break;
		case 11: tmp =  22; break;
		case 12: tmp =  24; break;
		case 13: tmp =  26; break;
		case 14: tmp =  28; break;
		case 15: tmp =  30; break;
		case 16: tmp =  1; break;
		case 17: tmp =  3; break;
		case 18: tmp =  5; break;
		case 19: tmp =  7; break;
		case 20: tmp =  9; break;
		case 21: tmp =  11; break;
		case 22: tmp =  13; break;
		case 23: tmp =  15; break;
		case 24: tmp =  17; break;
		case 25: tmp =  19; break;
		case 26: tmp =  21; break;
		case 27: tmp =  23; break;
		case 28: tmp =  25; break;
		case 29: tmp =  27; break;
		case 30: tmp =  29; break;
		case 31: tmp =  31; break;
	}
	return num - tmp_1 + tmp;
}

inline size_t itr_inv(const size_t &num)
{
	//size_t tmp = num%32;
	//return (tmp/16)*(2*(tmp-16)+1) + (1-(tmp/16))*(2*tmp) + 32*(num/32);
	
	size_t tmp;
	size_t tmp_1 = num % VN;
	switch (tmp_1){
		case 0: tmp = 0; break;
		case 1: tmp = 16; break;
		case 2: tmp = 1; break;
		case 3: tmp = 17; break;
		case 4: tmp = 2; break;
		case 5: tmp =  18; break;
		case 6: tmp =  3; break;
		case 7: tmp =  19; break;
		case 8: tmp =  4; break;
		case 9: tmp =  20; break;
		case 10: tmp =  5; break;
		case 11: tmp =  21; break;
		case 12: tmp =  6; break;
		case 13: tmp =  22; break;
		case 14: tmp =  7; break;
		case 15: tmp =  23; break;
		case 16: tmp =  8; break;
		case 17: tmp =  24; break;
		case 18: tmp =  9; break;
		case 19: tmp =  25; break;
		case 20: tmp =  10; break;
		case 21: tmp =  26; break;
		case 22: tmp =  11; break;
		case 23: tmp =  27; break;
		case 24: tmp =  12; break;
		case 25: tmp =  28; break;
		case 26: tmp =  13; break;
		case 27: tmp =  29; break;
		case 28: tmp =  14; break;
		case 29: tmp =  30; break;
		case 30: tmp =  15; break;
		case 31: tmp =  31; break;
	}
	return num - tmp_1 + tmp;
//	return VN*(num/VN) + tmp;
}


bool jud(const int &num){
	bool b1 = (num > -1);
	bool b2 = (num < (int)pc->points.size());
	if(b1&&b2)
		return true;
	else
		return false;
}

Vector3f vec(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
	Vector3f rsl=Vector3f::Zero();
	rsl[0] = p2.x - p1.x;
	rsl[1] = p2.y - p1.y;
	rsl[2] = p2.z - p1.z;
	return rsl;
}


void copy(const sensor_msgs::PointCloud &pc1, pcl::PointCloud<pcl::PointXYZRGB> &pc2)
{
	size_t sz = pc1.points.size();
	pc2.resize(sz);

	for(size_t i=0;i<sz;i++){
		pc2.points[i].x = pc1.points[i].x;
		pc2.points[i].y = pc1.points[i].y;
		pc2.points[i].z = pc1.points[i].z;
		pc2.points[i].r = 0.0;
		pc2.points[i].g = 0.0;
		pc2.points[i].b = 1.0;
	}
}

void copy(const sensor_msgs::PointCloud &pc1, pcl::PointCloud<pcl::PointNormal> &pc2)
{
	size_t sz = pc1.points.size();
	pc2.resize(sz);

	for(size_t i=0;i<sz;i++){
		pc2.points[i].x = pc1.points[i].x;
		pc2.points[i].y = pc1.points[i].y;
		pc2.points[i].z = pc1.points[i].z;
	}
}

bool alignment(const Vector3f &vec_q, const Vector3f &vec_n){
	float tmp = vec_q.dot(vec_n);

	if(tmp>0.0)
		return true;
	else
		return false;
}

void pc_callback(sensor_msgs::PointCloud2::Ptr msg)
{
	ros::Time tm = msg->header.stamp;
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_pc_c (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointNormal> pcl_pc_n;
	pcl::PointCloud<pcl::PointNormal> pcl_pc_nd;
	pcl::fromROSMsg(*msg, *pc);

	size_t i_end = pc->points.size() * VR;


//	pcl_pc_c->points.resize(i_end);
	pcl_pc_n.points.resize(i_end / skip);
	pcl_pc_nd.points.resize(i_end / skip);

	size_t index = 0;

#pragma omp parallel for
	//for(size_t i=0;i<pc->points.size();i+=10){//for azimuth_estimation 	(high speed)
	for(size_t i=0;i<i_end;i+=skip){//for azimuth_estimation 	(high speed)

		size_t ii = itr(i);

		float q_x, q_y, q_z;
		q_x = pc->points[ii].x;
		q_y = pc->points[ii].y;
		q_z = pc->points[ii].z;
		float tmp_d = sqrt(q_x*q_x + q_y*q_y + q_z*q_z);


		MatrixXf A(MatrixXf::Zero(3,(2*HNN+1)*(2*VNN+1)) );
		unsigned int cnt = 0;
		Vector3f cent(Vector3f::Zero(3));
//		Vector3f vec_q(Vector3f::Zero(3));

		Vector3f vec_q;
		vec_q << pc->points[ii].x, pc->points[ii].y, pc->points[ii].z;

		for(short int j=-VNN;j<=VNN;j++){
			for(short int k=-HNN;k<=HNN;k++){


				if(jud(i+j+32*k)){
					size_t num_tmp = itr(i+j+32*k);
					size_t v_idx = itr(i+j);
					Vector3f v_tmp_1, v_tmp_2;

					v_tmp_1 <<
						pc->points[ii].x-pc->points[v_idx].x,
						pc->points[ii].y-pc->points[v_idx].y,
						pc->points[v_idx].z-pc->points[num_tmp].z;		//本点と垂点ベクトル

					v_tmp_2 <<
						pc->points[v_idx].x-pc->points[num_tmp].x,
						pc->points[v_idx].y-pc->points[num_tmp].y,
						pc->points[v_idx].z-pc->points[num_tmp].z;	//垂点と疑点ベクトル

					float norm_1 = v_tmp_1.norm();
					float norm_2 = v_tmp_2.norm();
					if( (norm_1 < 0.1*pow(tmp_d,1.8)) && ( norm_2 < 0.5 ) && ( fabs((int)(i%VN)-(int)((i+j+32*k)%VN)) ) <= VNN ){		//2013.11.10変更後
						A.col(cnt) << pc->points[num_tmp].x, pc->points[num_tmp].y, pc->points[num_tmp].z;
						cent += A.col(cnt);
						cnt ++;
					}
				}

			}
		}


		cent /= (float)cnt;
		for(size_t a = 0;a<cnt;a++){
			A.col(a) = A.col(a) - cent;
		}


		JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);
		Matrix3f U = svd.matrixU();
		Vector3f S = svd.singularValues();

		Vector3f vec_n;
		vec_n = U.col(2);
		vec_n.normalize();		//正規化されていないとの指摘の元追加した。2013.11.11
		if(alignment(vec_q, vec_n)) vec_n *= -1.0;

		float value=1.0;
		value *= S(2)/S(1);
		value /= (float)cnt/((2*(float)HNN)*(2*(float)VNN));
		if(value > 1.0) value = 1.0;


		pcl::PointNormal p1;
		p1.x = pc->points[ii].x;
		p1.y = pc->points[ii].y;
		p1.z = pc->points[ii].z;
		p1.normal_x = -vec_n(0);
		p1.normal_y = -vec_n(1);
		p1.normal_z = -vec_n(2);
		//p1.curvature = 3.0*S(2)/(S(0) + S(1) + S(2));
		p1.curvature = value;


		pcl::PointNormal p2;
		p2.x = -vec_n(0);
		p2.y = -vec_n(1);
		p2.z = -vec_n(2);
		p2.curvature = 3.0*S(2)/(S(0) + S(1) + S(2));


/*
		pcl::PointXYZRGBA p3;
		p3.r = 255*fabs(vec_n(0));
		p3.g = 255*fabs(vec_n(1));
		p3.b = 255*fabs(vec_n(2));
		p3.rgba = value;
*/
//		pcl_pc_c->points[i] = p3;
		pcl_pc_n.points[index] = p1;
		pcl_pc_nd.points[index] = p2;
		index ++;

//		pcl_pc_c->points.push_back(p3);
//		pcl_pc_n.points.push_back(p1);
//		pcl_pc_nd.points.push_back(p2);

	}

	//cout << index << " / " << i_end / skip << endl;

//	pcl::toROSMsg(*pcl_pc_c, ros_pc_c);
//	ros_pc_c.header.frame_id = "/velodyne";
//	ros_pc_c.header.stamp = ros::Time::now();

	pcl::toROSMsg(pcl_pc_n, ros_pc_n);
	ros_pc_n.header.frame_id = "/velodyne";
	//ros_pc_n.header.stamp = ros::Time::now();
	ros_pc_n.header.stamp = tm;

	//cout << index << " / " << i_end / skip << endl;
	pcl::toROSMsg(pcl_pc_nd, ros_pc_nd);
	ros_pc_nd.header.frame_id = "/velodyne";
	ros_pc_nd.header.stamp = ros::Time::now();


	//cout << index << " / " << i_end / skip << endl;
	//	pc.header.frame_id = "/velodyne";
	//	pc.header.stamp = ros::Time::now();

//	pub_2.publish(ros_pc_c);
	pub_3.publish(ros_pc_n);
	pub_4.publish(ros_pc_nd);
	//	pub.publish(pc);
}

using namespace std;
int main (int argc, char** argv)
{
	cout << "start" << endl;
	ros::init(argc, argv, "NormalEstimationForVelodyne");
	ros::NodeHandle n;
	ros::Rate roop(2);
	ros::Subscriber sub = n.subscribe("/velodyne_points",1,pc_callback);
	//	pub = n.advertise<sensor_msgs::PointCloud>("perfect_velodyne",1);
	pub_2 = n.advertise<sensor_msgs::PointCloud2>("perfect_velodyne/color",1);
	pub_3 = n.advertise<sensor_msgs::PointCloud2>("perfect_velodyne/normal",1);
	pub_4 = n.advertise<sensor_msgs::PointCloud2>("perfect_velodyne/normal_sphere",1);
	//while(ros::ok()){	
	ros::spin();
	//	roop.sleep();
	//}
	return 0;
}
