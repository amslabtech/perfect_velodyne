//file: normal_estimation.cpp
//package: perfect_velodyne
//author: shogo shimizu
//last update: 2013.08.03
//2013.11.11    推定された法線を正規化した。

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
#include <visualization_msgs/Marker.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <omp.h>

//#define HNN 5
//#define HNN 7
#define VNN 1
#define VN 32
//#define THRESH_D 0.05 // not bekizyou
#define THRESH_D 0.2

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZI Point;
typedef pcl::PointXYZINormal PointN;
typedef pcl::PointCloud<PointN> CloudN;
typedef pcl::PointCloud<PointN>::Ptr CloudNPtr;
typedef pcl::PointCloud<Point>  Cloud;
typedef pcl::PointCloud<Point>::Ptr  CloudPtr;

sensor_msgs::PointCloud2 ros_pc_c;
sensor_msgs::PointCloud2 ros_pc_n;
sensor_msgs::PointCloud2 ros_pc_nd;
visualization_msgs::Marker mk;

CloudPtr pc (new Cloud);
ros::Publisher pub;
ros::Publisher pub_2;
ros::Publisher normal_cloud_pub;
ros::Publisher gaussian_sphere_pub;
ros::Publisher d_gaussian_sphere_pub;
ros::Publisher pub_mk;

size_t skip = 5;
int HNN = 7;
float VR = 0.839 * 0.60;
float MIN_RANGE = 0.2f;
float MAX_RANGE = 120.0f;

float DENS;
float CURV;
float vector_horizon;
float vector_vertical;
int DISP;


template <class T>
void getParam(ros::NodeHandle &n, string param, T &val){
    string str;
    if(!n.hasParam(param)){
        cout << param << " don't exist." << endl;
    }

    if(!n.getParam(param, str)){
        cout << "NG" << endl;
    }
    std::stringstream ss(str);
    T rsl;
    ss >> rsl;
    val = rsl;
    cout << param << " = " << str << endl;
}

bool getParams(ros::NodeHandle &n){
    getParam(n, "HNN", HNN);
    getParam(n, "MIN_RANGE", MIN_RANGE);
    getParam(n, "MAX_RANGE", MAX_RANGE);
    getParam(n, "skip", skip);
    getParam(n, "VR", VR);
    getParam(n, "vector_horizon", vector_horizon);
    getParam(n, "vector_vertical", vector_vertical);
    getParam(n, "DENS", DENS);
    getParam(n, "CURV", CURV);
    getParam(n, "DISP", DISP);
    return true;
}

void rm_zero(CloudN &pc){
    CloudN tmp;
    tmp.points.clear();
    for(size_t i=0;i<pc.points.size();i++){
        float x = pc.points[i].x;
        float y = pc.points[i].y;
        float z = pc.points[i].z;
        // float c = pc.points[i].curvature;
        float d = sqrt(x*x + y*y + z*z);

        if((d < MAX_RANGE) && (d  > MIN_RANGE)){
            tmp.points.push_back(pc.points[i]);
        }
    }
    pc.points.clear();
    pc = tmp;
}

void rm_zero_nd(CloudN &pc){
    CloudN tmp;
    tmp.points.clear();
    for(size_t i=0;i<pc.points.size();i++){
        float x = pc.points[i].x;
        float y = pc.points[i].y;
        float z = pc.points[i].z;
        float c = pc.points[i].curvature;
        float d = sqrt(x*x + y*y + z*z);

        if((d < MAX_RANGE) && (d > MIN_RANGE) && (c < CURV))
        {
            tmp.points.push_back(pc.points[i]);
        }
    }
    //cout << pc.points.size() ;
    pc.points.clear();
    pc = tmp;

    //cout << " -> " << pc.points.size() << endl ;
}


void init_zero(CloudN &in){
    for(size_t i=0;i<in.points.size();i++){
        in.points[i].x = 0.0;
        in.points[i].y = 0.0;
        in.points[i].z = 0.0;
    }
}

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
    //  return VN*(num/VN) + tmp;
}


inline bool jud(const int &num){
    bool b1 = (num >= 0);
    bool b2 = (num < (int)pc->points.size());
    if(b1&&b2)
        return true;
    else
        return false;
}

inline bool is_valid(float n){
    float a = fabs(n);
    if((a>MIN_RANGE)&&(a<MAX_RANGE))
        return true;
    else
        return false;
}
/*
   inline bool is_valid(Vector3f v){
   return is_valid(v.norm());
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

void copy(const sensor_msgs::PointCloud &pc1, CloudN &pc2)
{
    size_t sz = pc1.points.size();
    pc2.resize(sz);

    for(size_t i=0;i<sz;i++){
        pc2.points[i].x = pc1.points[i].x;
        pc2.points[i].y = pc1.points[i].y;
        pc2.points[i].z = pc1.points[i].z;
    }
}

bool alignment(const Vector3f &p_q, const Vector3f &vec_n){
    float tmp = p_q.dot(vec_n);

    if(tmp>0.0)
        return true;
    else
        return false;
}

void disp(const CloudN &pc)
{
    mk.header.frame_id = "/velodyne";
    mk.header.stamp = ros::Time();
    mk.ns = "vector";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.action = visualization_msgs::Marker::ADD;
    mk.scale.x = 0.01;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.color.a = 0.3;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    size_t sz = pc.points.size();
    mk.points.clear();
    for(size_t i=0;i<sz;i+=1){
        geometry_msgs::Point p1,p2;
        p1.x = pc.points[i].x;
        p1.y = pc.points[i].y;
        p1.z = pc.points[i].z;
        p2.x = pc.points[i].x-0.1*pc.points[i].normal_x;
        p2.y = pc.points[i].y-0.1*pc.points[i].normal_y;
        p2.z = pc.points[i].z-0.1*pc.points[i].normal_z;
        mk.points.push_back(p1);
        mk.points.push_back(p2);
    }
    pub_mk.publish( mk );
}


void pc_callback(sensor_msgs::PointCloud2::Ptr msg)
{
    std::cout << "callback" << std::endl;
    double start_time = ros::Time::now().toSec();
    ros::Time tm = msg->header.stamp;
    //  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_pc_c (new pcl::PointCloud<pcl::PointXYZRGBA>);
    CloudN pcl_pc_n;
    CloudN pcl_pc_nd;
    CloudN pcl_pc_final;
    CloudN pcl_pc_d_gauss;
    pcl::fromROSMsg(*msg, *pc);


    size_t i_end = pc->points.size() * VR;

    //  pcl_pc_c->points.resize(i_end);
    pcl_pc_n.points.resize(i_end / skip + 1);
    pcl_pc_nd.points.resize(i_end / skip + 1);
    pcl_pc_d_gauss.points.resize(i_end / skip + 1);

    init_zero(pcl_pc_n);



#pragma omp parallel for
    for(size_t i=0;i<i_end;i+=skip){

        size_t ii = itr(i);
        Vector3f p_q;
        p_q <<
            pc->points[ii].x,
            pc->points[ii].y,
            pc->points[ii].z;

        if(!is_valid(p_q.norm()))continue;
        float tmp_d = p_q.norm();

        if(tmp_d < MIN_RANGE)continue;

        MatrixXf A(MatrixXf::Zero(3,(2*HNN+1)*(2*VNN+1)) );
        unsigned int cnt = 0;
        //initialize of centroid point.
        Vector3f cent(Vector3f::Zero(3));


        for(int j=-VNN;j<=VNN;j++){
            //本点と垂点ベクトル
            Vector3f v_v;
            Vector3f p_v;
            if(!jud((int)i+j)) continue;
            size_t v_idx = itr(i+j);
            p_v <<
                pc->points[v_idx].x,
                pc->points[v_idx].y,
                pc->points[v_idx].z;
            //v_v : vector_vertical
            v_v = p_v - p_q;

            float norm_vv = v_v.norm();
            //if(! (norm_vv < 0.1*pow(tmp_d,1.8)) ) continue;
            if(! (norm_vv < vector_vertical*0.0216*pow(tmp_d,1.8967)) ) continue;
            //if(! (norm_vv < 3.0*0.0216*pow(tmp_d,1.8)) ) continue;

            for(int k=-HNN;k<=HNN;k++){

                if(!jud((int)i+j+32*k)) continue;
                size_t num_tmp = itr(i+j+32*k);
                Vector3f p_h;
                Vector3f v_h;

                //p_h : horizontal point
                p_h <<
                    pc->points[num_tmp].x,
                    pc->points[num_tmp].y,
                    pc->points[num_tmp].z;

                if(!is_valid(p_h.norm()))continue;
                //v_h : horizontal vector
                v_h << p_h - p_v;

                float norm_vh = v_h.norm();
                //if(( norm_vh < 0.5 ) && ( fabs((int)(i%VN)-(int)((i+j+32*k)%VN)) ) <= VNN ){}     //2013.11.10変更後
                if(( norm_vh < vector_horizon*tmp_d) && ( fabs((int)(i%VN)-(int)((i+j+32*k)%VN)) ) <= VNN ){        //2013.11.10変更後
                    A.col(cnt) = p_h;
                    cent += p_h;
                    cnt ++;
                }
            }
        }


        float density = (float)cnt/((2*VNN+1)*(2*HNN+1));
        if(density < DENS)continue;

        cent /= (float)cnt;
        for(size_t a = 0;a<cnt;a++){
            A.col(a) = A.col(a) - cent;
        }


        JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);
        Matrix3f U = svd.matrixU();
        Vector3f S = svd.singularValues();

        Vector3f vec_n;
        vec_n = U.col(2);
        //正規化処理@2013.11.11
        vec_n.normalize();
        if(alignment(p_q, vec_n)) vec_n *= -1.0;

        float value=1.0;
        value *= S(2)/S(1);
        value /= (float)cnt/((2*(float)HNN)*(2*(float)VNN));
        if(value > 1.0) value = 1.0;

        PointN p1;

        //曲率を与える
        p1.curvature = 3.0*S(2)/(S(0) + S(1) + S(2));
        //p1.curvature = value;
        //p1.curvature = density;



        //              float dd = (p_q - cent).norm();
        //              if(dd>1.50)continue;


        float w = 1.0 - pow(p1.curvature, 0.3);
        w *= w;
        Vector3f cor = vec_n.dot(cent - p_q) * vec_n;

        p_q = p_q + w * cor;

        p1.x = p_q(0);
        p1.y = p_q(1);
        p1.z = p_q(2);
        p1.intensity = pc->points[ii].intensity;
        p1.normal_x = -vec_n(0);
        p1.normal_y = -vec_n(1);
        p1.normal_z = -vec_n(2);

        PointN p2;
        p2.x = -vec_n(0);
        p2.y = -vec_n(1);
        p2.z = -vec_n(2);
        p2.intensity = pc->points[ii].intensity;
        p2.normal_x = vec_n(0);
        p2.normal_y = vec_n(1);
        p2.normal_z = vec_n(2);
        p2.curvature = 3.0*S(2)/(S(0) + S(1) + S(2));

        PointN p3;
        double inner_product = p_q.dot(-vec_n);
        p3.x = -vec_n(0) * inner_product;
        p3.y = -vec_n(1) * inner_product;
        p3.z = -vec_n(2) * inner_product;
        p3.intensity = p1.intensity;
        p3.normal_x = p1.normal_x;
        p3.normal_y = p1.normal_y;
        p3.normal_z = p1.normal_z;
        p3.curvature = p1.curvature;

        size_t index = i / skip;
        //      pcl_pc_c->points[index] = p3;
        pcl_pc_n.points[index] = p1;
        pcl_pc_nd.points[index] = p2;
        pcl_pc_d_gauss.points[index] = p3;
        //if( (abs(vec_n(2))<0.99999) && (p1.x != 0.0) && (p1.y != 0.0) && (p1.z != 0.0) )

        //      pcl_pc_n.points.push_back(p1);

        //      pcl_pc_c->points.push_back(p3);
        //      pcl_pc_n.points.push_back(p1);
        //      pcl_pc_nd.points.push_back(p2);


    }


    rm_zero(pcl_pc_n);
    rm_zero_nd(pcl_pc_nd);
    rm_zero_nd(pcl_pc_d_gauss);

    //  pcl::toROSMsg(*pcl_pc_c, ros_pc_c);
    //  ros_pc_c.header.frame_id = "/velodyne";
    //  ros_pc_c.header.stamp = ros::Time::now();

    if(DISP) disp(pcl_pc_n);

    pcl::toROSMsg(pcl_pc_n, ros_pc_n);
    pcl_pc_n.points.clear();
    ros_pc_n.header.frame_id = "/velodyne";
    //ros_pc_n.header.stamp = ros::Time::now();
    ros_pc_n.header.stamp = tm;

    //cout << index << " / " << i_end / skip << endl;
    pcl::toROSMsg(pcl_pc_nd, ros_pc_nd);
    ros_pc_nd.header.frame_id = "/velodyne";
    ros_pc_nd.header.stamp = ros::Time::now();


    //cout << index << " / " << i_end / skip << endl;
    //  pc.header.frame_id = "/velodyne";
    //  pc.header.stamp = ros::Time::now();

    //  pub_2.publish(ros_pc_c);
    normal_cloud_pub.publish(ros_pc_n);
    gaussian_sphere_pub.publish(ros_pc_nd);

    sensor_msgs::PointCloud2 d_normal;
    pcl::toROSMsg(pcl_pc_d_gauss, d_normal);
    d_normal.header = msg->header;
    d_gaussian_sphere_pub.publish(d_normal);
    //  pub.publish(pc);
    std::cout << "time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;;
}

using namespace std;
int main (int argc, char** argv)
{
    cout << "NormalEstimator_refine start" << endl;
    ros::init(argc, argv, "NormalEstimationForVelodyne");
    ros::NodeHandle n;
    ros::Rate roop(2);
    getParams(n);
    ros::Subscriber sub = n.subscribe("/velodyne_points",1,pc_callback);
    //  pub = n.advertise<sensor_msgs::PointCloud>("perfect_velodyne",1);
    //pub_2 = n.advertise<sensor_msgs::PointCloud2>("perfect_velodyne/color",1);
    normal_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/normal",1);
    gaussian_sphere_pub = n.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/normal_sphere",1);
    d_gaussian_sphere_pub = n.advertise<sensor_msgs::PointCloud2>("/perfect_velodyne/d_gaussian_sphere",1);
    pub_mk = n.advertise<visualization_msgs::Marker>("/perfect_velodyne/normal_vector",1);
    //while(ros::ok()){
    ros::spin();
    //  roop.sleep();
    //}
    return 0;
}

