//file: curvature_estimation.cpp
//package: perfect_velodyne
//author: shogo shimizu
//last update: 2013.08.03

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


//#define NN 10
#define NN 3
#define VN 32

using namespace std;
using namespace Eigen;

const bool occ = true; 		//オクルージョン境界の判別を　する/しない　(true/false)
const float param_1 = 50.0;	//円歪度に対するゲイン
const float param_2 = 0.7;	//傾度に対するゲイン
const float param_3 = 30.0;	//オクルージョン境界に対する係数（大きいほどオクル境界で検出が起こる）

sensor_msgs::PointCloud2 cloud_crv_ros;
pcl::PointCloud<pcl::PointXYZI> cloud_crv_pcl;
ros::Publisher pub;


size_t itr_inv(size_t num)
{
	size_t tmp;
	switch (num%VN){
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
	return VN*(num/VN) + tmp;
	/*
	size_t tmp = 32*(num/32);
	return 32*((num-tmp)/32)+16*((num-tmp)%2)+(num-tmp)/2+tmp;
*/	
}

int itr(int num)
{
	int tmp = num%32;
	return (tmp/16)*(2*(tmp-16)+1) + (1-(tmp/16))*(2*tmp) + 32*(num/32);
}

/*
bool jud(int num){
	bool b1 = (num > -1);
	bool b2 = (num < (int)pc.points.size());
	if(b1&&b2)
		return true;
	else
		return false;
}
*/
Vector3f vec(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
	Vector3f rsl=Vector3f::Zero();
	rsl[0] = p2.x - p1.x;
	rsl[1] = p2.y - p1.y;
	rsl[2] = p2.z - p1.z;
	return rsl;
}

std::vector<unsigned char> color(float val)
{
	std::vector<unsigned char> col;
	return col;
}

float hc(float num, float limit)
{
	if(num > limit)return limit;
	else return num;
}

void copy(sensor_msgs::PointCloud pc1, pcl::PointCloud<pcl::PointXYZRGB> &pc2)
{
	
	for(size_t i=0;i<pc1.points.size();i++){
		pcl::PointXYZRGB p;
		p.x = pc1.points[i].x;
		p.y = pc1.points[i].y;
		p.z = pc1.points[i].z;
		p.r = 0.0;
		p.g = 0.0;
		p.b = 1.0;
		
		pc2.push_back(p);
	}
}

void copy(sensor_msgs::PointCloud pc1, pcl::PointCloud<pcl::PointNormal> &pc2)
{
	
	for(size_t i=0;i<pc1.points.size();i++){
		pcl::PointNormal p;
		p.x = pc1.points[i].x;
		p.y = pc1.points[i].y;
		p.z = pc1.points[i].z;
		
		pc2.push_back(p);
	}
}

float alignment(Vector3f vec_q, Vector3f vec_n){
	float tmp = vec_q.dot(vec_n);
	float rsl = 1.0;

	if(tmp>0.0){
		 rsl *= -1.0;
	}else{
		 rsl *= 1.0;
	}
//	Vector3f tmp_v1, tmp_v2;
//	tmp_v1 = vec_q;
//	tmp_v2 = vec_n;
//	tmp_v1.normalize();
//	tmp_v2.normalize();
//	tmp = (tmp_v1.cross(tmp_v2)).norm();
//	if( (tmp > 0.86)&&(vec_n(2) < 0.0) ) rsl*= -1.0;
//	if( vec_n(2) < -0.3) rsl *= -1.0;

	return rsl;
}

float range(pcl::PointXYZI p){
	float x = p.x;
	float y = p.y;
	float z = p.z;
	return sqrt(x*x + y*y + z*z);
}

float calc(float r[]){
	float rsl = 0.0;
	float cnt = 0.0;
	for(int i=1;i<2*NN;i++){
		if((r[i] != -1)&&(r[i-1] != -1)){
			rsl += fabs(r[i] - r[i-1]);
			cnt ++;
		}
	}
	rsl /= cnt;
	rsl *= param_1;
	if(occ)
	if(rsl > param_3) rsl = 0.0;
	return rsl;
}

float head_cut(float n){
	if( n > 1.0)
	n=1.0;
	return n;
}

void push(pcl::PointXYZI p, float c){
	pcl::PointXYZI tmp;
	tmp = p;
	tmp.intensity = c;
	cloud_crv_pcl.points.push_back(tmp);
}


void pc_callback(sensor_msgs::PointCloud2::Ptr msg)
{
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*msg, *cloud);
	for(int i=0;i<(int)cloud->points.size();i++){
		int ii = itr(i);
		float dist = range(cloud->points[ii]);
		if( dist>= 2.5){
			float ranges[2*NN];			//マクロな箱
			for(int j=0;j<2*NN;j++){
				int tmp = ii + VN*(j-NN);
				if( (tmp >= 0) && (tmp <= (int)cloud->points.size()) && (range(cloud->points[tmp]) >= 0.1) )	//もし有効なナンバなら
					ranges[j] = range(cloud->points[tmp]);
				else
					ranges[j] = -1;
			}

			float vv = 0.0;
			if(i%31){
				int jj = itr(i+1);
				Vector3f p;
				p << cloud->points[jj].x-cloud->points[ii].x, cloud->points[jj].y-cloud->points[ii].y, cloud->points[jj].z-cloud->points[ii].z; 
				p.normalize();
				//cout << "p=" << endl << p << endl;
				vv = param_2*fabs(p(2));
			}

			float crv = calc(ranges);
			float w = 1/dist;
			if(w > 0.2) w = 0.2;
			crv *= w;
			crv += vv;
			crv = head_cut(crv);
			push(cloud->points[ii],crv);
		}
	}
	pcl::toROSMsg(cloud_crv_pcl, cloud_crv_ros);
	cloud_crv_pcl.points.clear();

	cloud_crv_ros.header.frame_id = "/velodyne";
	cloud_crv_ros.header.stamp = ros::Time::now();
	pub.publish(cloud_crv_ros);

}

using namespace std;
int main (int argc, char** argv)
{
	cout << "start" << endl;
	ros::init(argc, argv, "CurvatureEstimationForVelodyne");
  	ros::NodeHandle n;
	ros::Rate roop(2);
	ros::Subscriber sub = n.subscribe("/velodyne_points",1,pc_callback);
	pub = n.advertise<sensor_msgs::PointCloud2>("/curvature",1);
	ros::spin();
	
	return 0;
}
