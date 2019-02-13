#ifndef PERFECT_VELODYNE_H
#define PERFECT_VELODYNE_H

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace PERFECT_VELODYNE{

class NormalEstimation{
    private:
        ros::NodeHandle nh;

        ros::Subscriber sub;

        ros::Publisher pub_cloud;
        ros::Publisher pub_sphere;

        int VNN;
        int VN;
        double THRESH_D;

        int skip;
        double skip_;	
        int HNN;
        int HNN_VNN;
        double HNN_VNN_;
        double HNN2VNN2_;
        double VR;
        double MIN_RANGE;
        double MIN_RANGE2;
        double MAX_RANGE;
        double MAX_RANGE2;

        double DENS;
        double CURV;
        double vector_horizon;
        double vector_vertical;
        int DISP;

    public:
        NormalEstimation();

        void callback(const sensor_msgs::PointCloud2::ConstPtr&);

        size_t itr(const size_t&);
};


} // namespace

#endif
