// package : perfect_velodyne
// src : perfect_velodyne.cpp
// 
// fast normal estimation for velodyne hdl32e

#include <perfect_velodyne.h>

namespace PERFECT_VELODYNE{

NormalEstimation::NormalEstimation()
    : nh("~")
{

    sub = nh.subscribe("/velodyne_points", 1, &NormalEstimation::callback, this);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/normal", 1);
    pub_sphere = nh.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/sphere", 1);

    VNN = 1;
    VN = 32;
    THRESH_D = 0.2;

    nh.param<int>("HNN", HNN, 5);
    nh.param<double>("MIN_RANGE", MIN_RANGE, 0.2);
    nh.param<double>("MAX_RANGE", MAX_RANGE, 120);
    nh.param<int>("skip", skip, 5);
    nh.param<double>("VR", VR, 0.839*0.60);
    nh.param<double>("vector_horizon", vector_horizon, 0);
    nh.param<double>("vector_vertical", vector_vertical, 0);
    nh.param<double>("DENS", DENS, 0);
    nh.param<double>("CURV", CURV, 0);
    nh.param<int>("DISP", DISP, 0);

	HNN_VNN = (2*HNN+1)*(2*VNN+1);
	HNN_VNN_ = 1.0/HNN_VNN;
	HNN2VNN2_ = 1.0/((2*(float)HNN)*(2*(float)VNN));
	MIN_RANGE2 = (MIN_RANGE*MIN_RANGE);
	MAX_RANGE2 = (MAX_RANGE*MAX_RANGE);
	skip_ = 1.0/skip;
}

void NormalEstimation::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pc);

    pcl::PointCloud<pcl::PointXYZINormal> pcl_pc_n;
    pcl::PointCloud<pcl::PointXYZINormal> pcl_pc_nd;
    pcl::PointCloud<pcl::PointXYZINormal> pcl_pc_final;

    size_t i_end = pc->points.size();

    pcl_pc_n.points.resize(i_end * skip_ + 1);
    pcl_pc_nd.points.resize(i_end * skip_ + 1);

}

size_t NormalEstimation::itr(const size_t &num)
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


} // namespace PERFECT_VELODYNE
