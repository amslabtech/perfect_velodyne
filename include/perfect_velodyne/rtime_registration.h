#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>


#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

class Registration
{
private:
	Matrix<double,3,6> model;
	MatrixXd	   query;
	Matrix3d	   accum;
	Matrix3d 	   state;
	vector<double>	  weight;
	vector<double>   val_weight;
	vector<int>	    pair;
	size_t		       n;
	size_t		   val_n;
	double w_max;
	double rely;

public:
	Registration ();
	~Registration (){};
	void check();
	void setQuery(sensor_msgs::PointCloud2ConstPtr cloud);
	void rotate();
	void rotate_inv();
	void pairing();
	void process();
	void output(sensor_msgs::PointCloud2 &rsl);
	void getPair(visualization_msgs::Marker &marker);
	void getPose(sensor_msgs::Imu &pose);
	void reset(){accum = Matrix3d::Identity();}
	void specialFeedBack(Matrix3d rot){accum = rot;}
};

Registration::Registration (){
	model << 1,0,0,-1, 0, 0,
		 0,1,0, 0,-1, 0,
		 0,0,1, 0, 0,-1;
	state = Matrix3d::Identity();
	accum = Matrix3d::Identity();
	rely = 0.0;
}

void Registration::check(){
	double tmp;
	Vector3d v1 = accum.col(0);
	Vector3d v2 = accum.col(1);
	Vector3d v3 = accum.col(2);
	tmp = v1.dot(v2.cross(v3));
	cout << "v1 x v2 x v3 = " << tmp << endl;
	if(tmp < 0.0){
		accum.col(2) *= -1.0;
		cout << "corrected"<<endl;
	}
	cout << "size of query is " << query.cols() << endl;
	for(size_t i=0;i<(size_t)weight.size();i++){
		cout.width(5);
		cout << weight[i] << " " ;
	}
	cout << endl;
	for(size_t i=0;i<(size_t)weight.size();i++){
		cout.width(5);
		cout << pair[i] << " " ;
	}
	cout << endl << "n = " << (int)n << endl << "--------" << endl;

/************信頼度を調査するための追加項目***********/
	double reliability = 0.0;
	double max = 0.0;
	for(size_t i=0;i<pair.size();i++){
		if(pair[i]!=-1){
			if(max < weight[i])max = weight[i];
			//cc ++;
			reliability += weight[i];
		}
	}
	rely = reliability;
	rely -= 1.0;
	if(rely < 0.0)rely = 0.0;
	reliability -= max;	//cluster0があまりにもでかいので
	cout << "RELIABILITY = " << reliability << endl;
	FILE *fp = fopen ("reliability.csv","a");
	fprintf(fp,"%f\n",reliability);
	fclose(fp);
/**********************************************/
}

void Registration::setQuery(sensor_msgs::PointCloud2ConstPtr cloud){
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*cloud, *tmp);
	size_t num = tmp->points.size();
	query.resize(3,num);
	weight.clear();
	n = 0;
	pair.clear();
	for(size_t i=0;i<num;i++){
		query(0,i) = tmp->points[i].x;
		query(1,i) = tmp->points[i].y;
		query(2,i) = tmp->points[i].z;
		query.col(i).normalize();
		weight.push_back(tmp->points[i].intensity);
		pair.push_back(-1);
	}
}

void Registration::rotate(){
	query = accum * query;
}

void Registration::rotate_inv(){
	query = accum.transpose() * query;
}

void Registration::pairing(){
	val_weight.clear();
	w_max = 1.0;
	for(size_t i=0;i<(size_t)query.cols();i++){
		double m = 0.90;
		for(size_t j=0;j<6;j++){ //最尤な対応点を見つける
			if((query.col(i)).dot(model.col(j)) > m){
				pair[i] = j;
			}
		}
		if(pair[i]!=-1){	//有効な最尤対応点が見つかったら
			n++;
			val_weight.push_back(weight[i]);
			if(w_max < weight[i]) w_max = weight[i];
		}
	}
}
		

void Registration::process(){
	cout << "accum = " << endl << accum << endl;
	Matrix3d TMP;
	MatrixXd X(3,n);
	MatrixXd Y(3,n);
	for(size_t i=0;i<n;i++){
		if(pair[i]!=-1){
			X.col(i) = query.col(i);
			Y.col(i) = model.col(pair[i]);
		}
	}

	size_t l_n = val_weight.size();
	MatrixXd W = MatrixXd::Zero(l_n,l_n);
	for(size_t i=0;i<l_n;i++){
		W(i,i) = val_weight[i];
	}
	W = W / w_max;
	cout.width(6);
	cout << "W = " << endl << W << endl;

	TMP = X * W * Y.transpose();
	//TMP = X * Y.transpose();
	JacobiSVD<Matrix3d> svd(TMP, Eigen::ComputeFullU | Eigen::ComputeFullV);
	MatrixXd U,V;

	U = svd.matrixU();
	V = svd.matrixV();

	Matrix3d RSL;
	RSL = V * U.transpose();
	state = V * U.transpose();

	double mat_norm = 0.0;
	Matrix3d D = Matrix3d::Identity();
	D = D-state;
	for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
		mat_norm += sqrt(D(i,j)*D(i,j));
	cout << "mat_norm = " << mat_norm << endl;
	if( (mat_norm > 1.0) || (isnan(mat_norm)) ){
		state = Matrix3d::Identity();
		cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
		//exit(1);
	}


	accum = state * accum;

	cout << "state = " << endl << RSL << endl;
	cout << "accum = " << endl << accum << endl;

	}

void Registration::output(sensor_msgs::PointCloud2 &rsl){

	MatrixXd TMP = state * query;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZI>);
	for(size_t i=0;i<(size_t)TMP.cols();i++){
		pcl::PointXYZI p;
		p.x = TMP(0,i);
		p.y = TMP(1,i);
		p.z = TMP(2,i);
		tmp1->points.push_back(p);
	}
	pcl::toROSMsg(*tmp1, rsl);
	rsl.header.frame_id = "/velodyne";
	double yaw;
	//rol = acos(accum(2,2));
	//yaw = acos( (accum(2,1))/sin(rol) );
	yaw = atan2(accum(0,0),accum(1,0));
//	if(yaw >  2*M_PI)yaw-=2*M_PI;
//	if(yaw <     0.0)yaw+=2*M_PI;
//	yaw *=180/M_PI;
	//////////
	yaw = yaw*180/M_PI -90.0;
	if(yaw < -180)yaw += 360.0;
	if(yaw >  180)yaw -= 360.0;
	//////////
//	Adouble data = yaw;	//if you write the .csv file
	cout << "yaw = " << yaw << "[deg]" << endl;
	yaw += 180;
	for(double i=0;i<360;i+=1){
		if((int)i%90==0)cout << "*";
		else if(i<yaw)cout << "|";
		else cout << "-";
	}
	cout << endl;

	
/*
	FILE* fp = fopen("data.csv","a");
	fprintf(fp,"%f\n",data);
	fclose(fp);
*/
}


void Registration::getPair(visualization_msgs::Marker &marker){
	marker.points.resize(0);
	marker.ns = "perfect_velodyne/pair";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	for(size_t i=0;i<pair.size();i++){
		if(pair[i]!=-1){
			geometry_msgs::Point p;
			p.x = query(0,i);
			p.y = query(1,i);
			p.z = query(2,i);
			marker.points.push_back(p);
			p.x = model(0,pair[i]);
			p.y = model(1,pair[i]);
			p.z = model(2,pair[i]);
			marker.points.push_back(p);
			marker.scale.x = 0.02;
			marker.color.r = 0.2;
			marker.color.g = 0.2;
			marker.color.b = 0.2;
			marker.color.a = 1.0;
		}
	}
}




void Registration::getPose(sensor_msgs::Imu &pose){
	double yaw;
	//rol = acos(accum(2,2));
	yaw = atan2(accum(0,0),accum(1,0));
//	yaw = yaw*180/M_PI;
	yaw = yaw*180/M_PI-90.0;
	if(yaw < -180)yaw += 360.0;
	if(yaw >  180)yaw -= 360.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = -yaw;
	pose.orientation.w = 0.0;
	pose.angular_velocity.x = 0.0;
	pose.angular_velocity.y = 0.0;
	pose.angular_velocity.z = 0.0;
	pose.linear_acceleration.x = 0.0;
	pose.linear_acceleration.y = 0.0;
	pose.linear_acceleration.z = 0.0;
	for(size_t i=0;i<9;i++){
		pose.orientation_covariance[i] = 0.0;
		pose.angular_velocity_covariance[i] = 0.0;
		pose.linear_acceleration_covariance[i] = 0.0;
	}
	pose.orientation_covariance[0] = rely;
}












