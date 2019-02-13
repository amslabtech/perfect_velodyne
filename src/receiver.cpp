//file: receiver.cpp
//package: perfect_velodyne
//author: shogo shimizu
//last update: 2013.08.03

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodyneScan.h>

velodyne_msgs::VelodyneScan org;
sensor_msgs::PointCloud pc;

float range[181*12*32];
float ang[181*12];
unsigned int a_shift=0;
int a_tmp=0;
/*
void f(float r[32])
{
	float vang = -(30.0 + 2.0/3.0);
	for(size_t i=0;i<32;i++){
		pc.points[i].x = r[i]*cos(vang);
		pc.points[i].z = r[i]*sin(vang);
		vang += 1.0/3.0;
	}
		
}

void f(unsigned char c)
{
	f();
	trans(ang,)

void f(bool dat[8], unsigned short int val)
{
	dat[0] = val / 128;
	if(dat[0])val-=128;
	dat[1] = val / 64;
	if(dat[1])val-=64;
	dat[2] = val / 32;
	if(dat[2])val-=32;
	dat[3] = val / 16;
	if(dat[3])val-=16;
	dat[4] = val / 8;
	if(dat[4])val-=8;
	dat[5] = val / 4;
	if(dat[5])val-=4;
	dat[6] = val / 2;
	if(dat[6])val-=2;
	dat[7] = val / 1;
	if(dat[7])val-=1;
}
*/

void f()
{
	geometry_msgs::Point32 pp;
	for(size_t i=0;i<(181*12);i++){
		//cout <<ang[i]<<endl;
		for(size_t j=0;j<32;j++){
			float vang = -(30.0+2.0/3.0) + (float)j*(4.0/3.0);
			vang = vang * M_PI/180.0;
			//cout << range[32*i + j] << endl;
			pp.x = range[32*i+j] *cos(vang)*cos(ang[i]);
			pp.y = range[32*i+j] *cos(vang)*sin(ang[i]);
			pp.z = range[32*i+j] *sin(vang);
			pc.points.push_back(pp);
		}
	}
}

void velodyne_callback(velodyne_msgs::VelodyneScan::Ptr msg)
{
	pc.points.clear();
	//cout << "//////////////////////////velodyne_calback///////////////////////" << endl;
	//cout << msg->packets[0].data.size() << endl;
	for(size_t i=0;i<181;i++){
		for(size_t j=0;j<12;j++){
			unsigned char a_hd = msg->packets[i].data[100*j + 1];
			unsigned char a_tl = msg->packets[i].data[100*j + 2];
			//unsigned long int tmp = a_tl*256 + a_hd;
			unsigned long int tmp = (a_tl<<8) + a_hd;
			//ang[12*i + j] = (float)(a_tl * 256 + a_hd)/36000.0/100.0 + 12.0*(float)(12*i+j)/100.0;
			//cout << tmp ;
			if((a_tmp - tmp)>0){
				a_shift ++;
				a_shift = a_shift%36000;
			}
			a_tmp = tmp;
		//	cout << tmp << endl;
			//ang[12*i+j] = (float)(a_tl * 256 + a_hd)/36000.0/100.0 + (float)a_shift;
			ang[12*i+j] = (float)(tmp + a_shift*36000)/36000/36000*360;
			//cout << ang[12*i+j] << endl;
			//ang[12*i + j] *= 1.05*M_PI/2.0;
			//a_shift += 12.0/180.0;

			for(size_t k=0;k<32;k++){
				unsigned char d_hd = msg->packets[i].data[4 + 100*j + 3*k];
				unsigned char d_tl = msg->packets[i].data[4 + 100*j + 3*k + 1];
				//cout << (float)(d_tl * 256 + d_hd) * 0.002 << endl ;
				range[12*32*i + 32*j + 16*(k%2) + k/2] = (float)((d_tl<<8) + d_hd) * 0.002 ;
			}
		}
	}
	for(int i=0;i<181*12;i++){
		ang[i] = 19.68*ang[i]*M_PI/180.0;
	}
	f();

	
/*
	unsigned short int val;
	for(size_t i=4;i<100;i+=3){
		val = (int)msg->packets[0].data[i];
		f(dat,val);				
		for(int j=0;j<8;j++) cout << dat[j];
		cout << " ";	
	
		val = (int)msg->packets[0].data[i+1];
		f(dat,val);				
		for(int j=0;j<8;j++) cout << dat[j];
		cout << endl;

		unsigned short int head = msg->packets[0].data[i];
		unsigned short int tail = msg->packets[0].data[i+1];
		cout << "h:" << head << " t:" << tail << " " ;
		cout << (float)(tail * 256 + head) * 0.002 << endl ;
		cout << " ";
		
	}
	v_pc[16*(i%2) + i] = (float)(tail * 256 + head) * 0.002;
	cout << endl;
*/	org = *msg;
}

using namespace std;
int main (int argc, char** argv)
{
	ros::init(argc, argv, "PerfectVelodyne_receiver");
  	ros::NodeHandle n;
	ros::Rate roop(20);
	ros::Subscriber sub = n.subscribe("/velodyne_packets",1,velodyne_callback);
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("perfect_velodyne", 1);
	pc.header.frame_id = "/world";
	while(ros::ok()){
		pc.header.stamp=ros::Time::now();
		pub.publish(pc);
		//pc2_pub.publish(pc2);
		//m_pub.publish(marker);
		ros::spinOnce();
		roop.sleep();
	}

	//pcl::io::savePCDFileBinary("detection.pcd", *cloud_s);
	return 0;
}
