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
	pub_marker = nh.advertise<visualization_msgs::Marker>("/perfect_velodyne/normal_vector",1);

    VNN = 1;
    VN = 32;
    THRESH_D = 0.2;

    nh.param<int>("HNN", HNN, 3l);
    nh.param<double>("MIN_RANGE", MIN_RANGE, 0.2);
    nh.param<double>("MAX_RANGE", MAX_RANGE, 120);
    nh.param<int>("skip", skip, 1);
    nh.param<double>("VR", VR, 1.0068);
    nh.param<double>("vector_horizon", vector_horizon, 0.1);
    nh.param<double>("vector_vertical", vector_vertical, 1.5);
    nh.param<double>("DENS", DENS, 0.5);
    nh.param<double>("CURV", CURV, 0.1);
    nh.param<int>("DISP", DISP, 0l);

	HNN_VNN = (2*HNN+1)*(2*VNN+1);
	HNN_VNN_ = 1.0/HNN_VNN;
	HNN2VNN2_ = 1.0/((2*(float)HNN)*(2*(float)VNN));
	MIN_RANGE2 = (MIN_RANGE*MIN_RANGE);
	MAX_RANGE2 = (MAX_RANGE*MAX_RANGE);
	skip_ = 1.0/skip;
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

inline bool NormalEstimation::is_valid(float n){
	float a = fabs(n);
	if((a>MIN_RANGE)&&(a<MAX_RANGE))
		return true;
	else
		return false;
}


inline bool NormalEstimation::jud(const int &num, size_t size){
	bool b1 = (num >= 0);
	bool b2 = (num < (int)size);
	if(b1&&b2)
		return true;
	else
		return false;
}


bool NormalEstimation::alignment(const Eigen::Vector3f &p_q, const Eigen::Vector3f &vec_n){
	float tmp = p_q.dot(vec_n);

	if(tmp>0.0)
		return true;
	else
		return false;
}

void NormalEstimation::rm_zero(pcl::PointCloud<pcl::PointXYZINormal> &pc){
    pcl::PointCloud<pcl::PointXYZINormal> tmp;
	tmp.points.clear();
	size_t i, pc_size = pc.points.size();
	float x;
	float y;
	float z;
	float d;
	for(i=0; i<pc_size; ++i){
		x = pc.points[i].x;
		y = pc.points[i].y;
		z = pc.points[i].z;
		d = x*x + y*y + z*z;
		if((d < MAX_RANGE2) && (d  > MIN_RANGE2)){
			tmp.points.push_back(pc.points[i]);
		}
	}
	pc.points.clear();
	pc = tmp;
}

void NormalEstimation::rm_zero_nd(pcl::PointCloud<pcl::PointXYZINormal> &pc){
    pcl::PointCloud<pcl::PointXYZINormal> tmp;
	tmp.points.clear();
	size_t i, pc_size = pc.points.size();
	float x;
	float y;
	float z;
	float c;
	float d;
	for(i=0; i<pc_size; ++i){
		x = pc.points[i].x;
		y = pc.points[i].y;
		z = pc.points[i].z;
		c = pc.points[i].curvature;
		d = x*x + y*y + z*z;

		if( (d > 0.01) && (c < 0.18) ){
			tmp.points.push_back(pc.points[i]);
		}
	}
	pc.points.clear();
	pc = tmp;
}

void NormalEstimation::disp(const pcl::PointCloud<pcl::PointXYZINormal>&pc)
{
    visualization_msgs::Marker mk;
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

    pub_marker.publish(mk);
}


void NormalEstimation::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ros::Time time = msg->header.stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pc);

    pcl::PointCloud<pcl::PointXYZINormal> pcl_pc_n;
    pcl::PointCloud<pcl::PointXYZINormal> pcl_pc_nd;

    size_t i_end = pc->points.size();

    pcl_pc_n.points.resize(i_end * skip_ + 1);
    pcl_pc_nd.points.resize(i_end * skip_ + 1);

#pragma omp parallel for
    for(size_t i=0;i<pc->points.size(); i++)
    {
        size_t ii = itr(i);
        Eigen::Vector3f p_q;
        p_q << pc->points[ii].x,
               pc->points[ii].y,
               pc->points[ii].z;


        if(!is_valid(p_q.norm()))continue;
        float tmp_d = p_q.norm();
        float vvtd = vector_vertical*0.0216*pow(tmp_d,1.8967);
        float vhpq = vector_horizon*p_q.norm();
        int ivn = (int)(i%VN);

        if(p_q.norm() < MIN_RANGE)continue;

        Eigen::MatrixXf A(Eigen::MatrixXf::Zero(3,HNN_VNN) );
        unsigned int cnt = 0;
        Eigen::Vector3f cent(Eigen::Vector3f::Zero(3));

        for(int j=-VNN;j<=VNN;j++){
            int i_j = i+j;
            Eigen::Vector3f v_v;
            Eigen::Vector3f p_v;
            if(!jud(i_j, pc->points.size())) continue;
            size_t v_idx = itr(i_j);
            p_v <<
                pc->points[v_idx].x,
                pc->points[v_idx].y,
                pc->points[v_idx].z;
            v_v = p_v - p_q;

            float norm_vv = v_v.norm();
            if(! (norm_vv < vvtd) ) continue;

            for(int k=-HNN;k<=HNN;k++){

                int ij32k = i_j+32*k;

                if(!jud(ij32k, pc->points.size())) continue;
                size_t num_tmp = itr(ij32k);
                Eigen::Vector3f p_h;
                Eigen::Vector3f v_h;

                p_h <<
                    pc->points[num_tmp].x,
                    pc->points[num_tmp].y,
                    pc->points[num_tmp].z;

                if(!is_valid(p_h.norm()))continue;
                v_h << p_h - p_v;

                float norm_vh = v_h.norm();
                if(( norm_vh < vhpq ) && ( abs(ivn-(int)((ij32k)%VN)) ) <= VNN ){	// 20151104
                    A.col(cnt) = p_h;
                    cent += p_h;
                    cnt ++;
                }
            }
        }

		float density = (float)cnt*HNN_VNN_;	// 20151104
		if(density < DENS)continue;

		cent /= (float)cnt;
		for(size_t a = 0;a<cnt;a++){
			A.col(a) = A.col(a) - cent;
		}

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Vector3f S = svd.singularValues();

        Eigen::Vector3f vec_n;
		vec_n = U.col(2);
		vec_n.normalize();
		if(alignment(p_q, vec_n)) vec_n *= -1.0;

		float value=1.0;
		value *= S(2)/S(1);
		value /= (float)cnt*HNN2VNN2_;
		if(value > 1.0) value = 1.0;

        pcl::PointXYZINormal p1;

		float w = 1.0 - pow(p1.curvature, 0.3);
		w *= w;
        Eigen::Vector3f cor = vec_n.dot(cent - p_q) * vec_n;
		p_q = p_q + w * cor;

		p1.x = p_q(0);
		p1.y = p_q(1);
		p1.z = p_q(2);
		p1.normal_x = -vec_n(0);
		p1.normal_y = -vec_n(1);
		p1.normal_z = -vec_n(2);
        p1.curvature = 3.0*S(2)/(S(0) + S(1) + S(2));
        p1.intensity = pc->points[ii].intensity;

        pcl::PointXYZINormal p2;
		p2.x = -vec_n(0);
		p2.y = -vec_n(1);
		p2.z = -vec_n(2);
		p2.normal_x = 0.0;
		p2.normal_y = 0.0;
		p2.normal_z = 1.0;
		p2.curvature = 3.0*S(2)/(S(0) + S(1) + S(2));
        p2.intensity = pc->points[ii].intensity;

		size_t index = i * skip_;
		pcl_pc_n.points[index] = p1;
		pcl_pc_nd.points[index] = p2;
    }

	rm_zero(pcl_pc_n);
	rm_zero_nd(pcl_pc_nd);

	if(DISP) disp(pcl_pc_n);

    sensor_msgs::PointCloud2 ros_pc_n;
    sensor_msgs::PointCloud2 ros_pc_nd;

	pcl::toROSMsg(pcl_pc_n, ros_pc_n);
	pcl_pc_n.points.clear();
	ros_pc_n.header.frame_id = "/velodyne";
	ros_pc_n.header.stamp = time;

	pcl::toROSMsg(pcl_pc_nd, ros_pc_nd);
	ros_pc_nd.header.frame_id = "/velodyne";
	ros_pc_nd.header.stamp = ros::Time::now();

    pub_cloud.publish(ros_pc_n);
    pub_sphere.publish(ros_pc_nd);
}


} // namespace PERFECT_VELODYNE
