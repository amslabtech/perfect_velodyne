#ifndef PERFECT_VELODYNE_H
#define PERFECT_VELODYNE_H

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <visualization_msgs/Marker.h>
#include <omp.h>

namespace PERFECT_VELODYNE{

class NormalEstimation{
    private:
        ros::NodeHandle nh;

        ros::Subscriber sub;

        ros::Publisher pub_cloud;
        ros::Publisher pub_sphere;
        ros::Publisher pub_marker;

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
        inline bool is_valid(float);
        inline bool jud(const int&, size_t size);
        bool alignment(const Eigen::Vector3f&, const Eigen::Vector3f&);
        void rm_zero(pcl::PointCloud<pcl::PointXYZINormal>&);
        void rm_zero_nd(pcl::PointCloud<pcl::PointXYZINormal>&);
        void disp(const pcl::PointCloud<pcl::PointXYZINormal>&);
};


} // namespace

#endif
