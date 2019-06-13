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

#include <Eigen/Core>
#include <Eigen/SVD>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <velodyne_msgs/VelodyneScan.h>


#define HNN 7
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

size_t itr(size_t num)
{
	size_t tmp = num%32;
	return (tmp/16)*(2*(tmp-16)+1) + (1-(tmp/16))*(2*tmp) + 32*(num/32);
}


bool jud(int num){
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

void pc_callback(sensor_msgs::PointCloud2::Ptr msg)
{
//	cout << "pc_callback" << endl;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_pc_c (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointNormal> pcl_pc_n;
	pcl::PointCloud<pcl::PointNormal> pcl_pc_nd;
	pcl::fromROSMsg(*msg, *pc);
//	pcl::fromROSMsg(*msg, *pcl_pc_c);
	//sensor_msgs::convertPointCloud2ToPointCloud(*msg,pc);
	//copy(pc, pcl_pc_c);	//sensor_msgs::PointCoud -> pcl::PointCloud<pcl::PointXYZRGB> only value of coordinate.
	//copy(pc, pcl_pc_n);	//sensor_msgs::PointCoud -> pcl::PointCloud<pcl::PointXYZNormal> only value of coordinate.
	//copy(pc, pcl_pc_nd);	//sensor_msgs::PointCoud -> pcl::PointCloud<pcl::PointXYZNormal> only value of coordinate.

		
//	for(size_t i=0;i<pc->points.size();i++){//for fine normal_estimation	(high resolution)
	for(size_t i=0;i<pc->points.size();i+=10){//for azimuth_estimation 	(high speed)
	//for(size_t i=0;i<100;i++){
		
		size_t ii = itr(i);

		float q_x, q_y, q_z;
		q_x = pc->points[ii].x;
		q_y = pc->points[ii].y;
		q_z = pc->points[ii].z;
		float tmp_d = sqrt(q_x*q_x + q_y*q_y + q_z*q_z);


		MatrixXf A(MatrixXf::Zero(3,(2*HNN+1)*(2*VNN+1)) );
		unsigned int cnt = 0;
		Vector3f cent(Vector3f::Zero(3));
		Vector3f vec_q(Vector3f::Zero(3));
		Vector3f u=Vector3f::Zero(), v=Vector3f::Zero();

		vec_q << pc->points[ii].x, pc->points[ii].y, pc->points[ii].z;

		for(int j=-VNN;j<=VNN;j++){
		//	Vector3f vertical_vec;
			//size_t vertical_num;
		//	if(jud(i+j));
		//	vertical_vec << pc->points[itr(i+j)].x, pc->points[itr(i+j)].y, pc->points[itr(i+j)].z;
			for(int k=-HNN;k<=HNN;k++){
				

				if(jud(i+j+32*k)){
					size_t num_tmp = itr(i+j+32*k);
					Vector3f v_tmp_1, v_tmp_2;
					float x,y,z,norm_1,norm_2;
					
					v_tmp_1 << pc->points[ii].x-pc->points[itr(i+j)].x, pc->points[ii].y-pc->points[itr(i+j)].y, pc->points[itr(i+j)].z-pc->points[num_tmp].z;		//本点と垂点ベクトル
					v_tmp_2 << pc->points[itr(i+j)].x-pc->points[num_tmp].x, pc->points[itr(i+j)].y-pc->points[num_tmp].y, pc->points[itr(i+j)].z-pc->points[num_tmp].z;	//垂点と疑点ベクトル
					
					/*
					x = pc->points[itr(i)].x - pc->points[num_tmp].x;
					y = pc->points[itr(i)].y - pc->points[num_tmp].y;
					z = pc->points[itr(i)].z - pc->points[num_tmp].z;
					*/
/*
					x = pc->points[itr(i+j)].x - pc->points[num_tmp].x;
					y = pc->points[itr(i+j)].y - pc->points[num_tmp].y;
					z = pc->points[itr(i+j)].z - pc->points[num_tmp].z;

					v_tmp << 	pc->points[ii].x - pc->points[num_tmp].x,
							pc->points[ii].y - pc->points[num_tmp].y,
							pc->points[ii].z - pc->points[num_tmp].z;

*/
//					norm = v_tmp.norm();

					//norm = v_tmp.squaredNorm();
					norm_1 = v_tmp_1.norm();
					norm_2 = v_tmp_2.norm();
					//if( (norm_1 < 0.1*pow(tmp_d,1.8)) && ( norm_2 < 0.5 ) && ( fabs((int)(i%VN)-(int)((i+j+32*k)%VN)) ) <= VNN ){		//2013.11.10変更前
					if( (norm_1 < 0.1*pow(tmp_d,1.8)) && ( norm_2 < 0.5 ) && ( fabs((int)(i%VN)-(int)((i+j+32*k)%VN)) ) <= VNN ){		//2013.11.10変更後
					//if( ( norm < THRESH_D*pow(tmp_d,2.0) ) && ( fabs((int)(i%VN)-(int)((i+j+32*k)%VN)) ) <= VNN ){
					//if(norm < THRESH_D){
					//	A.col(cnt) = v_tmp;
						A.col(cnt) << pc->points[num_tmp].x, pc->points[num_tmp].y, pc->points[num_tmp].z;
						cent += A.col(cnt);
						cnt ++;
					}
/*
					size_t tmp = itr(i+j+32*k);
					size_t tmp_1 = tmp%32;
					if( (j!=VNN)&&(tmp_1!=31) )
						v += vec(pc->points[tmp], pc->points[tmp+2]);
					if(k!=HNN)
						u += vec(pc->points[tmp], pc->points[tmp+32]);
*/
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
		vec_n = vec_n * alignment(vec_q, vec_n);


		//if(vec_n(2) < 0.0)vec_n(2)*=-1.0;	//禁忌

/*
		Matrix3f Q;
		HouseholderQR<MatrixXf> qr(A);	
		Q = qr.householderQ();
*/

		//v += vec(pc->points[itr(i)], pc->points[itr(i+1)]);
		//u += vec(pc->points[itr(i)], pc->points[itr(i)+32]);
		//u.normalize();
		//v.normalize();
		//Vector3f n = v.cross(u);
		//n.normalize();
		//v.normalize();
		//u.normalize();
		float value=1.0;
		//value = (float)i/pc->points.size();
	//	value = 12.0*S(2)/(S(0) + S(1) + S(2));
		value *= S(2)/S(1);
		value /= (float)cnt/((2*(float)HNN)*(2*(float)VNN));
		//value = ((float)itr(i%32)/32.0) ;
		//value = fabs(pc->points[itr(i)].z)/3.0;
		//value = ;
//		value = value / sqrt(tmp_d);
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



		pcl::PointXYZRGBA p3;
		p3.r = 255*fabs(vec_n(0));
		p3.g = 255*fabs(vec_n(1));
		p3.b = 255*fabs(vec_n(2));
		p3.rgba = value;

/*
		pcl_pc_c->points[ii].x = -vec_n(0);
		pcl_pc_c->points[ii].y = -vec_n(1);
		pcl_pc_c->points[ii].z = -vec_n(2);
*/
/*
		pcl_pc_c.points[ii].r = 255*fabs(U(0,2));
		pcl_pc_c.points[ii].g = 255*fabs(U(1,2));
		pcl_pc_c.points[ii].b = 255*fabs(U(2,2));
*/
/*
		pcl_pc_c.points[ii].r = 255*fabs(Q(0,2));
		pcl_pc_c.points[ii].g = 255*fabs(Q(1,2));
		pcl_pc_c.points[ii].b = 255*fabs(Q(2,2));
*/
/*
		pcl_pc_c->points[ii].r = 255*(1.0-hc(value,1.0));
		pcl_pc_c->points[ii].g = 255*(1.0-hc(value,1.0));
		pcl_pc_c->points[ii].b = 255*(1.0-hc(value,1.0));
*/


//		if(fabs(vec_q.dot(vec_n)) > 0.000001){
		
		pcl_pc_c->points.push_back(p3);
		
		pcl_pc_n.points.push_back(p1);/*
		pcl_pc_n.points[ii].normal_x = vec_n(0);
		pcl_pc_n.points[ii].normal_y = vec_n(1);
		pcl_pc_n.points[ii].normal_z = vec_n(2);
		pcl_pc_n.points[ii].curvature = 3.0*S(2)/(S(0) + S(1) + S(2));*/

		pcl_pc_nd.points.push_back(p2);/*
		pcl_pc_nd.points[ii].x = -vec_n(0);
		pcl_pc_nd.points[ii].y = -vec_n(1);
		pcl_pc_nd.points[ii].z = -vec_n(2);
		pcl_pc_nd.points[ii].curvature = 3.0*S(2)/(S(0) + S(1) + S(2));*/
		
//		}


//cout << n << endl << endl;
	}


/*
		size_t t = itr(i);
		if( ((t%32) != itr(0))&&((t%32) !=itr(1))&&((t%32) !=itr(3)) ){
			pc->points[t].x=pc->points[t].y=pc->points[t].z=0.0;
		}
	}*/
	pcl::toROSMsg(*pcl_pc_c, ros_pc_c);
	ros_pc_c.header.frame_id = "/velodyne";
	ros_pc_c.header.stamp = ros::Time::now();

	pcl::toROSMsg(pcl_pc_n, ros_pc_n);
	ros_pc_n.header.frame_id = "/velodyne";
	ros_pc_n.header.stamp = ros::Time::now();

	pcl::toROSMsg(pcl_pc_nd, ros_pc_nd);
	ros_pc_nd.header.frame_id = "/velodyne";
	ros_pc_nd.header.stamp = ros::Time::now();


//	pc.header.frame_id = "/velodyne";
//	pc.header.stamp = ros::Time::now();
	
	pub_2.publish(ros_pc_c);
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
