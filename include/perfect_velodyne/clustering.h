//2013.11.08にpcl_testから転載した。
//コメントを追加した以外本文に変更はしてない。
//変更した場合はここに記載すること。

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

#include <iostream>

using namespace Eigen;
using namespace std;

class Cluster
{
public:
	Vector3f getCentroid(){return centroid;}
	void setCentroid(Vector3f v){centroid = v;}
	void addMem(size_t m){member.push_back(m);}
	size_t getSize(){return member.size();}
	vector<size_t> getMem(){return member;}
	void showMem(){cout << member.size() << endl;}
	void showCentroid(){cout << centroid;}
	void setWeight(float w){weight = w;}
	float getWeight(void){return weight;}

	size_t getMember(size_t i){return member[i];}

	Cluster(){centroid = Vector3f::Zero(); weight = 0.0;}
	Cluster(Vector3f c){centroid = c; weight=0.0;}
	~Cluster(){};

private:
	Vector3f centroid;
	vector<size_t> member;
	float weight;
};

/////////////////////inheritant////////////////////
class Clustering : public Cluster
{
public:
	void addPoint(size_t i, size_t t);
	void addTank(Vector3f p){tank.push_back(p);}
	void putTank(sensor_msgs::PointCloud2ConstPtr pc);
	void showTankSize(){cout << "Size of tank = " << tank.size() << endl;}
	void showTank();
	void showCentroid(size_t i);
	void showClusters();
	void process();
	void newCluster(Vector3f c, size_t n);
	void showClusterSize(){cout << endl << "number of clusters = " << clusters.size() << endl;}
	void setMaxNum(size_t n){max_num = n;}
	void setMinNum(size_t n){min_num = n;}
	void setMinMaxNum(size_t n, size_t m){min_num = n; max_num = m;}
	void setThreshold(float angle, float dist){THRESH_THETA = angle; THRESH_DIST = dist;}

	bool compare(Vector3f v1, Vector3f v2);

	size_t getClusterNum(){return clusters.size();}
	size_t getTankSize(){return tank.size();}

	void getCluster(size_t i, vector<Vector3f> &rsl);
	void getClusters(vector<Vector3f> &rsl);
	void getClusters(sensor_msgs::PointCloud2 &rsl);
	void rewriteTank(size_t i, Vector3f p){tank[i] = p;}

	Vector3f getCentroid(size_t i){return clusters[i].getCentroid();}
	
	Vector3f getTank(size_t i){return tank[i];}
	size_t getNMember(size_t i){return clusters[i].getSize();}
	size_t getMember(size_t i, size_t j){return clusters[i].getMember(j);}
	void rotate(Matrix3f rot);

	void getInfo(vector<sensor_msgs::PointCloud2> *master_pc, int &n);
	void calcWeight(void);
	void setWeight(size_t n, float w){clusters[n].setWeight(w);}
	float getWeight(size_t n){return clusters[n].getWeight();}

	Clustering();
	~Clustering(){};

private:
	vector<Cluster> clusters;
	Vector3f point;
	vector<Vector3f> tank;
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> master_cloud;
	size_t min_num;
	size_t max_num;
	float THRESH_THETA;
	float THRESH_DIST;
};

Clustering::Clustering(){
	min_num = 50;
	//min_num = 200;	///Ｄ館中央

	max_num = INT_MAX;
	THRESH_THETA = 0.6;
	THRESH_DIST = 0.1;
}

void Clustering::addPoint(size_t i, size_t t){	//i番目のクラスタにtankのt番目の点を追加する関数
	Vector3f tmp;
	if(clusters.size() != 0){
		tmp = (clusters[i].getSize() * clusters[i].getCentroid() + tank[t]) / (float)(clusters[i].getSize() + 1.0);
		clusters[i].setCentroid(tmp);
		clusters[i].addMem(t);
	}else{
		tmp = tank[t];
		Cluster c_tmp;
		c_tmp.setCentroid(tmp);
		c_tmp.addMem(t);
		clusters.push_back(c_tmp);
	}
} 

void Clustering::putTank(sensor_msgs::PointCloud2ConstPtr pc){
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromROSMsg(*pc, *cloud);
	for(size_t i=0;i<cloud->points.size();i++){
		if(cloud->points[i].curvature < 0.3){
			Vector3f p;
			p << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z; 
			addTank(p);
		}
	}
}

void Clustering::showTank(){	//tank内の点を全て表示
	for(size_t i=0;i<tank.size();i++){
		cout << tank[i] << endl;
	}
}

void Clustering::process(){	//クラスタリング処理
	for(size_t i=0;i<tank.size();i++){		//tankを渡るカウンタ変数i
		if(clusters.size() != 0){		//クラスタが一つも存在していなければelseに飛ぶ
			bool hit=false;
			for(size_t j=0;j<clusters.size();j++){	//クラスタを渡るカウンタ変数j
				if( compare(tank[i], clusters[j].getCentroid() ) ){
				//	cout << "hit." << endl;

				/////////////表示用//////////////
				//	pcl::PointXYZ p; p.x=tank[i](0); p.y=tank[i](1); p.z=tank[i](2);
				//	master_cloud[j]->points.push_back(p);
				///////////////////////////
				
					addPoint(j,i);
					hit = true;
					break;
				}
			}
			if(hit == false) newCluster(tank[i], i);
		}else{
			newCluster(tank[i], i);
		}
	}
//メンバ数によるフィルタリング
//	cout << "before " << clusters.size() << endl;
	vector<Cluster>::iterator itr = clusters.begin();
	while( itr!=clusters.end()){
		Cluster c = *itr;
		bool h1 = (c.getSize() < min_num);
		bool h2 = (c.getSize() > max_num);
		if( h1 || h2 ){
			itr = clusters.erase(itr);
		//	cout << "eliminate the cluster which have " << c.getSize() << " members." << endl;
		}else{
			itr ++;
		}
		
	}


//メンバ数によるフィルタリング（当初表示用の点群であったが、信頼性の算出に使用する点群でもある。）
/*
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator c_itr = master_cloud.begin();
	while( c_itr!=master_cloud.end()){
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud = *c_itr;
		bool h1 = (tmp_cloud->points.size() < min_num);
		bool h2 = (tmp_cloud->points.size() > max_num);
		if( h1 || h2 ){
			c_itr = master_cloud.erase(c_itr);
		//	cout << "eliminate the cluster which have " << c.getSize() << " members." << endl;
		}else{
			c_itr ++;
		}
		
	}
*/
////////////////////////////////////////////


//	cout << "生成されたクラス数は" << master_cloud.size() << "です。" << endl;
//	for(size_t i=0;i<master_cloud.size();i++){
//		cout << "\tsize of cluster " << i << " = " << master_cloud[i]->points.size() << endl;
//	}
//	cout << "after " << clusters.size() << endl;
//	cout.flush();
}



bool Clustering::compare(Vector3f v1, Vector3f v2){	//2ベクター間のcos距離を算出する（クラスタリング処理で用いられる）
	float d = fabs( v1.norm() - v2.norm() );	//depth 距離
	v1.normalize();
	v2.normalize();
	float p = v1.dot(v2);				//角度距離
//	cout << "p = " << p << endl;
	//return (d < 0.3) && (p > 0.95);		//for simulate point cloud(blonder)
//	return (d < 2.0) && (p > 0.95);	//for velodyne_snap_shot_first
//	return (d < 2.0) && (p > 0.60);	//for velodyne_snap_shot_second
//	return (d < 3.0) && (p > 0.95);	//for velodyne_snap_shot_four
	return (d < THRESH_DIST) && (p > THRESH_THETA);	//for azimuth_estimation
	//return (p > 0.95);
	//return (p > 0.80);
}

void Clustering::newCluster(Vector3f c, size_t n){	//新しいクラスタの生成
	Cluster tmp(c);
	tmp.addMem(n);
	clusters.push_back(tmp);
//	cout << "new cluster is created." << endl;

//	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointXYZ p;
//	p.x = tank[n](0); p.y = tank[n](1); p.z = tank[n](2);
//	tmp_cloud->points.push_back(p);
//	master_cloud.push_back(tmp_cloud);

}

void Clustering::showClusters(){			//クラスタ数と各クラスタのメンバ数を表示
	cout << endl;
	cout << "number of cluster = " << clusters.size() << endl;
	cout << "contents:" << endl;
	for(size_t i=0;i<clusters.size();i++){
		cout << "\tcluster" << i << " : " << clusters[i].getSize() << endl;
	}
}

void Clustering::getCluster(size_t i, vector<Vector3f> &rsl){
	if(i > (clusters.size() - 1) ){
		cout << "cluster of the number " << i << "does not exist." << endl;
	}else{
		rsl.clear();
		vector<size_t> tmp = clusters[i].getMem();
		for(size_t i=0;i<tmp.size();i++){
			rsl.push_back(tank[tmp[i]]);
		}
	}
}

void Clustering::getClusters(vector<Vector3f> &rsl){
	rsl.clear();
	for(vector<Cluster>::iterator itr=clusters.begin(); itr!=clusters.end(); itr++){
		rsl.push_back(itr->getCentroid());
	}
}

void Clustering::getClusters(sensor_msgs::PointCloud2 &rsl){
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZI>);

	size_t mx = 0;
	for(size_t i=0;i<clusters.size();i++)
	if(mx < getNMember(i)) mx = getNMember(i);

	size_t i=0;
	for(vector<Cluster>::iterator itr=clusters.begin(); itr!=clusters.end(); itr++){
		pcl::PointXYZI tmp_p;
		Vector3f tmp_v = itr->getCentroid();
		tmp_p.x = tmp_v(0);
		tmp_p.y = tmp_v(1);
		tmp_p.z = tmp_v(2);
		//tmp_p.intensity = (float)getNMember(i);
		tmp_p.intensity = getWeight(i);
		tmp_cloud->points.push_back(tmp_p);
		i++;
	}
	pcl::toROSMsg(*tmp_cloud, rsl);
}

void Clustering::showCentroid(size_t i){
	cout << endl << "Centroid of cluster " << i << " = " << endl;
	clusters[i].showCentroid();
	cout << endl;
}

void Clustering::rotate(Matrix3f rot){
	for(size_t i=0;i<clusters.size();i++){
		clusters[i].setCentroid( rot * clusters[i].getCentroid() ); //rotation of each centroid.
	}
}

void Clustering::getInfo(vector<sensor_msgs::PointCloud2> *master_pc, int &n){
	master_pc->clear();
	for(size_t i=0;i<clusters.size();i++){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		for(size_t j=0;j<getNMember(i);j++){
			Vector3f p = Vector3f::Zero();
			p = tank[getMember(i,j)];
			pcl::PointXYZ pcl_p;
			pcl_p.x = p(0);
			pcl_p.y = p(1);
			pcl_p.z = p(2);
			pcl_cloud->points.push_back(pcl_p);
		}
		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*pcl_cloud, ros_cloud);
		master_pc->push_back(ros_cloud);
	}
	n = clusters.size();
}

void Clustering::calcWeight(void){
	float offset = 1.0-THRESH_THETA;
	float max_sig = 0.0;
	float max_num = 0.0;
	for(size_t i=0;i<clusters.size();i++){
		float s=0.0;
		Vector3f c = getCentroid(i);
		c.normalize();
		for(size_t j=0;j<getNMember(i);j++){
			Vector3f q = tank[getMember(i,j)];
			q.normalize();
			float ds = (1.0 - c.dot(q)) / offset;
			s += ds*ds;

		}
		s /= (float)getNMember(i);
	//	cout << "sigma = " << s << endl;
		//if(max_sig < 1.0/s)max_sig = 1.0/s;
		//if(max_num < getNMember(i))max_num = getNMember(i);

		//setWeight(i, (float)getNMember(i));	//member weight
		setWeight(i, (float)getNMember(i)/s);	//hiblid
		//setWeight(i, 1.0/s);			//sigma

	//	cout << "weight = " << getWeight(i) << endl;
	}
	

	/**************NORMALIZATION*******************/
	float master_max = 0.0;
	for(size_t i=0;i<clusters.size();i++){
		if(master_max < getWeight(i))master_max = getWeight(i);
	}
	for(size_t i=0;i<clusters.size();i++){
		setWeight(i, getWeight(i)/master_max);
	}
	/*********************************************/

}
