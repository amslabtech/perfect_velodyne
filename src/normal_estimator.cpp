#include <ros/ros.h>

#include <Eigen/Dense>

#include <omp.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <visualization_msgs/Marker.h>

class NormalEstimator
{
public:
    typedef pcl::PointXYZI PointXYZI;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;
    typedef pcl::PointXYZINormal PointXYZIN;
    typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
    typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;

    NormalEstimator(void);

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void estimate_normal(CloudXYZINPtr&);
    bool validate_range(double);
    int get_ring_index_from_firing_order(int);
    int get_firing_order_from_ring_index(int);
    bool validate_ring(int);
    void get_gaussian_sphere(const CloudXYZINPtr&, CloudXYZINPtr&);
    void get_d_gaussian_sphere(const CloudXYZINPtr&, CloudXYZINPtr&);
    void publish_normal_marker(const CloudXYZINPtr&);
    void remove_invalid_points(CloudXYZINPtr&);
    void filter_curvature(CloudXYZINPtr&);
    void process(void);

private:
    double MAX_RANGE;
    double MIN_RANGE;
    int SKIP;
    int VERTICAL_POINTS;
    int HORIZONTAL_POINTS;
    int LAYER_NUM;
    double QUERY_RADIUS;
    double DENSITY;
    double GAUSSIAN_SPHERE_RADIUS;
    double MAX_CURVATURE_THRESHOLD;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Subscriber cloud_sub;
    ros::Publisher normal_cloud_pub;
    ros::Publisher gaussian_sphere_pub;
    ros::Publisher gaussian_sphere_filtered_pub;
    ros::Publisher d_gaussian_sphere_pub;
    ros::Publisher d_gaussian_sphere_filtered_pub;
    ros::Publisher normal_marker_pub;
};

NormalEstimator::NormalEstimator(void)
:local_nh("~")
{
    local_nh.param<double>("MAX_RANGE", MAX_RANGE, {120});
    local_nh.param<double>("MIN_RANGE", MIN_RANGE, {0.5});
    local_nh.param<int>("SKIP", SKIP, {1});
    local_nh.param<int>("VERTICAL_POINTS", VERTICAL_POINTS, {1});
    local_nh.param<int>("HORIZONTAL_POINTS", HORIZONTAL_POINTS, {5});
    local_nh.param<int>("LAYER_NUM", LAYER_NUM, {32});
    local_nh.param<double>("QUERY_RADIUS", QUERY_RADIUS, {0.5});
    local_nh.param<double>("DENSITY", DENSITY, {0.5});
    local_nh.param<double>("GAUSSIAN_SPHERE_RADIUS", GAUSSIAN_SPHERE_RADIUS, {1.0});
    local_nh.param<double>("MAX_CURVATURE_THRESHOLD", MAX_CURVATURE_THRESHOLD, {0.002});

    cloud_sub = nh.subscribe("/velodyne_points", 1, &NormalEstimator::cloud_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    normal_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/normal", 1);
    gaussian_sphere_pub = nh.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/normal_sphere", 1);
    gaussian_sphere_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/normal_sphere/filtered", 1);
    d_gaussian_sphere_pub = nh.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/d_gaussian_sphere", 1);
    d_gaussian_sphere_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/d_gaussian_sphere/filtered", 1);
    normal_marker_pub = nh.advertise<visualization_msgs::Marker>("/perfect_velodyne/normal_vector", 1);

    std::cout << "MAX_RANGE: " << MAX_RANGE << std::endl;
    std::cout << "MIN_RANGE: " << MIN_RANGE << std::endl;
    std::cout << "SKIP: " << SKIP << std::endl;
    std::cout << "VERTICAL_POINTS: " << VERTICAL_POINTS << std::endl;
    std::cout << "HORIZONTAL_POINTS: " << HORIZONTAL_POINTS << std::endl;
    std::cout << "LAYER_NUM: " << LAYER_NUM << std::endl;
    std::cout << "QUERY_RADIUS: " << QUERY_RADIUS << std::endl;
    std::cout << "DENSITY: " << DENSITY << std::endl;
    std::cout << "GAUSSIAN_SPHERE_RADIUS: " << GAUSSIAN_SPHERE_RADIUS << std::endl;
    std::cout << "MAX_CURVATURE_THRESHOLD: " << MAX_CURVATURE_THRESHOLD<< std::endl;
}

void NormalEstimator::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "=== normal estimator === " << std::endl;
    double start_time = ros::Time::now().toSec();

    CloudXYZIPtr subscribed_cloud_ptr(new CloudXYZI);
    pcl::fromROSMsg(*msg, *subscribed_cloud_ptr);
    std::cout << "subscribed cloud size: " << subscribed_cloud_ptr->points.size() << std::endl;

    CloudXYZINPtr cloud_ptr(new CloudXYZIN);
    pcl::copyPointCloud(*subscribed_cloud_ptr, *cloud_ptr);

    estimate_normal(cloud_ptr);

    remove_invalid_points(cloud_ptr);

    std::cout << "output cloud size: " << cloud_ptr->points.size() << std::endl;
    normal_cloud_pub.publish(*cloud_ptr);

    if((gaussian_sphere_pub.getNumSubscribers() > 0) || (gaussian_sphere_filtered_pub.getNumSubscribers() > 0)){
        CloudXYZINPtr gaussian_sphere(new CloudXYZIN);
        get_gaussian_sphere(cloud_ptr, gaussian_sphere);
        gaussian_sphere_pub.publish(*gaussian_sphere);
        filter_curvature(gaussian_sphere);
        gaussian_sphere_filtered_pub.publish(*gaussian_sphere);
    }

    if((d_gaussian_sphere_pub.getNumSubscribers() > 0) || (d_gaussian_sphere_filtered_pub.getNumSubscribers() > 0)){
        CloudXYZINPtr d_gaussian_sphere(new CloudXYZIN);
        get_d_gaussian_sphere(cloud_ptr, d_gaussian_sphere);
        d_gaussian_sphere_pub.publish(*d_gaussian_sphere);
        filter_curvature(d_gaussian_sphere);
        d_gaussian_sphere_filtered_pub.publish(*d_gaussian_sphere);
    }

    if(normal_marker_pub.getNumSubscribers() > 0){
        publish_normal_marker(cloud_ptr);
    }

    std::cout << "time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
}

void NormalEstimator::estimate_normal(CloudXYZINPtr& cloud)
{
    std::cout << "estimate normal" << std::endl;
    const int SIZE = cloud->points.size();
    const int QUERY_SIZE = (2 * HORIZONTAL_POINTS + 1) * (2 * VERTICAL_POINTS + 1);

    #pragma omp parallel for
    for(int i=0;i<SIZE;i+=SKIP){
        Eigen::Vector3d p_q(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);// query point
        double distance_q = p_q.norm();
        if(!validate_range(distance_q)){
            continue;
        }
        int order = i % LAYER_NUM;
        int ring_index = get_ring_index_from_firing_order(order);
        int yaw_index = i - order;

        std::vector<double> x_data(QUERY_SIZE, 0);
        std::vector<double> y_data(QUERY_SIZE, 0);
        std::vector<double> z_data(QUERY_SIZE, 0);
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();

        int valid_point_count = 0;
        for(int j=-VERTICAL_POINTS;j<=VERTICAL_POINTS;j++){
            int v_ring_index = ring_index + j;
            if(!validate_ring(v_ring_index)){
                continue;
            }
            int v_index = yaw_index + get_firing_order_from_ring_index(v_ring_index);

            for(int k=-HORIZONTAL_POINTS;k<=HORIZONTAL_POINTS;k++){
                int h_index = v_index + k * LAYER_NUM;
                if(0 > h_index){
                    continue;
                }else if(h_index >= SIZE){
                    continue;
                }
                Eigen::Vector3d p_h(cloud->points[h_index].x, cloud->points[h_index].y, cloud->points[h_index].z);
                double distance_q_h = (p_h - p_q).norm();
                if(distance_q_h > QUERY_RADIUS){
                    continue;
                }
                x_data[valid_point_count] = p_h(0);
                y_data[valid_point_count] = p_h(1);
                z_data[valid_point_count] = p_h(2);
                sum += p_h;
                valid_point_count++;
            }
        }
        if(valid_point_count < QUERY_SIZE * DENSITY){
            continue;
        }
        // PCA
        Eigen::Vector3d ave = sum / (double)valid_point_count;
        double sigma_xx = 0;
        double sigma_xy = 0;
        double sigma_xz = 0;
        double sigma_yy = 0;
        double sigma_yz = 0;
        double sigma_zz = 0;
        for(int i=0;i<valid_point_count;i++){
            sigma_xx += (x_data[i] - ave(0)) * (x_data[i] - ave(0));
            sigma_xy += (x_data[i] - ave(0)) * (y_data[i] - ave(1));
            sigma_xz += (x_data[i] - ave(0)) * (z_data[i] - ave(2));
            sigma_yy += (y_data[i] - ave(1)) * (y_data[i] - ave(1));
            sigma_yz += (y_data[i] - ave(1)) * (z_data[i] - ave(2));
            sigma_zz += (z_data[i] - ave(2)) * (z_data[i] - ave(2));
        }
        Eigen::Matrix3d vcov;
        vcov << sigma_xx, sigma_xy, sigma_xz,
                sigma_xy, sigma_yy, sigma_yz,
                sigma_xz, sigma_yz, sigma_zz;
        vcov /= (double)valid_point_count;
        Eigen::EigenSolver<Eigen::Matrix3d> es(vcov);
        Eigen::Vector3d eigen_values = es.eigenvalues().real();
        Eigen::Matrix3d eigen_vectors = es.eigenvectors().real();

        int third_component_index = 0;// index of smallest eigen value
        third_component_index = eigen_values(third_component_index) > eigen_values(1) ? 1 : third_component_index;
        third_component_index = eigen_values(third_component_index) > eigen_values(2) ? 2 : third_component_index;

        Eigen::Vector3d normal_vector = eigen_vectors.col(third_component_index);
        normal_vector.normalize();

        if(normal_vector.dot(p_q) > 0.0){
            normal_vector = -normal_vector;
        }

        cloud->points[i].normal_x = normal_vector(0);
        cloud->points[i].normal_y = normal_vector(1);
        cloud->points[i].normal_z = normal_vector(2);
        cloud->points[i].curvature = eigen_values(third_component_index) / (eigen_values(0) + eigen_values(1) + eigen_values(2));
    }
}

bool NormalEstimator::validate_range(double range)
{
    return (MIN_RANGE <= range) && (range <= MAX_RANGE);
}

int NormalEstimator::get_ring_index_from_firing_order(int order)
{
    if(order % 2){
        return order / 2 + LAYER_NUM / 2;
    }else{
        return order / 2;
    }
}

int NormalEstimator::get_firing_order_from_ring_index(int index)
{
    const int LAYER_NUM_2 = LAYER_NUM / 2;
    if(index < LAYER_NUM_2){
        return 2 * index;
    }else{
        return 2 * (index - LAYER_NUM_2) + 1;
    }
}

bool NormalEstimator::validate_ring(int index)
{
    return (0 <= index) && (index < LAYER_NUM);
}

void NormalEstimator::get_gaussian_sphere(const CloudXYZINPtr& cloud, CloudXYZINPtr& gaussian_sphere)
{
    pcl::copyPointCloud(*cloud, *gaussian_sphere);
    for(auto& p : gaussian_sphere->points){
        p.x = GAUSSIAN_SPHERE_RADIUS * -p.normal_x;
        p.y = GAUSSIAN_SPHERE_RADIUS * -p.normal_y;
        p.z = GAUSSIAN_SPHERE_RADIUS * -p.normal_z;
    }
}

void NormalEstimator::get_d_gaussian_sphere(const CloudXYZINPtr& cloud, CloudXYZINPtr& d_gaussian_sphere)
{
    pcl::copyPointCloud(*cloud, *d_gaussian_sphere);
    for(auto& p : d_gaussian_sphere->points){
        double inner_product = p.x * p.normal_x + p.y * p.normal_y + p.z * p.normal_z;
        p.x = p.normal_x * inner_product;
        p.y = p.normal_y * inner_product;
        p.z = p.normal_z * inner_product;
    }
}

void NormalEstimator::publish_normal_marker(const CloudXYZINPtr& cloud)
{
    visualization_msgs::Marker marker;
    marker.header = pcl_conversions::fromPCL(cloud->header);
    marker.ns = "perfect_velodyne";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.a = 0.3;
    marker.color.g = 1.0;
    int size = cloud->points.size();
    marker.points.clear();
    for(int i=0;i<size;i++){
        geometry_msgs::Point p1,p2;
        p1.x = cloud->points[i].x;
        p1.y = cloud->points[i].y;
        p1.z = cloud->points[i].z;
        p2.x = cloud->points[i].x + 0.1 * cloud->points[i].normal_x;
        p2.y = cloud->points[i].y + 0.1 * cloud->points[i].normal_y;
        p2.z = cloud->points[i].z + 0.1 * cloud->points[i].normal_z;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }
    normal_marker_pub.publish(marker);
}

void NormalEstimator::remove_invalid_points(CloudXYZINPtr& cloud)
{
    int size = cloud->points.size();
    CloudXYZIN cloud_;
    cloud_.header = cloud->header;
    cloud_.points.clear();
    cloud_.points.reserve(cloud->points.size());
    for(int i=0;i<size;i++){
        double n_x = cloud->points[i].normal_x;
        double n_y = cloud->points[i].normal_y;
        double n_z = cloud->points[i].normal_z;
        // if normal is not estimated
        if(n_x * n_x + n_y * n_y + n_z * n_z == 0.0){
            continue;
        }

        double p_x = cloud->points[i].x;
        double p_y = cloud->points[i].y;
        double p_z = cloud->points[i].z;
        double distance = sqrt(p_x * p_x + p_y * p_y + p_z * p_z);
        if((MIN_RANGE < distance) && (distance < MAX_RANGE)){
            cloud_.points.push_back(cloud->points[i]);
        }
    }
    cloud->points.clear();
    *cloud = cloud_;
    cloud->width = cloud->points.size();
    cloud->height = 1;
}

void NormalEstimator::filter_curvature(CloudXYZINPtr& cloud)
{
    int size = cloud->points.size();
    CloudXYZIN cloud_;
    cloud_.header = cloud->header;
    cloud_.points.clear();
    cloud_.points.reserve(cloud->points.size());
    for(int i=0;i<size;i++){
        if(cloud->points[i].curvature < MAX_CURVATURE_THRESHOLD){
            cloud_.points.push_back(cloud->points[i]);
        }
    }
    cloud->points.clear();
    *cloud = cloud_;
    cloud->width = cloud->points.size();
    cloud->height = 1;
}

void NormalEstimator::process(void)
{
    std::cout << "=== normal estimator === " << std::endl;
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_estiamtor");
    NormalEstimator ne;
    ne.process();
    return 0;
}
