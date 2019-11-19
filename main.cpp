#include <iostream>	
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <Eigen/Dense>
#include "V3.hpp"
#include <vector>
#include <set>
#include <stack>
#include <algorithm>
using namespace std;
#ifndef PointType
#define PointType pcl::PointXYZRGBA
#endif

class Cluster
{
	public:
		Cluster(){
			idx_=INT_MAX;
			genus_=INT_MAX;
		}
		int idx_;
		int genus_;
};

int main(int argc,char** argv)
{
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);	
	if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1) 
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	
	// Init Variables
	vector<Cluster> cls;	
	cls.resize(cloud->points.size());
	int number_of_genus=0;
	pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
	kdtree->setInputCloud(cloud);

	// Inital ID of each item
	for(int i=0;i<cloud->points.size();i++){		
		cls[i].idx_=i;
	}
	cls[0].genus_=number_of_genus++;
	
	// 
	vector<int> buf;	
	for(int i=1;i<cloud->points.size();i++){				
		int idx_tmp=i;

		// judge whether it is belong to new genus
		if(cls[idx_tmp].genus_==INT_MAX){
			buf.clear();
			buf.push_back(idx_tmp);

			while(1){
			// find nearest neighbour
			int K=2;
			vector<int> pointIdxNKNSearch(K);
			vector<float> pointNKNSquaredDistance(K);
			kdtree->nearestKSearch (cloud->points[idx_tmp], K, pointIdxNKNSearch, pointNKNSquaredDistance);
			idx_tmp=pointIdxNKNSearch[1];

			// loop termination condition 1: this item has be classified into a genus
			if(cls[idx_tmp].genus_!=INT_MAX){
				// update all items belong to this genus
				for(int j=0;j<buf.size();j++){
					cls[buf[j]].genus_=cls[idx_tmp].genus_;
				}
				break;
			}
			// loop termination condition 2: find the closure
			if(find(buf.begin(),buf.end(),idx_tmp)!=buf.end()){
				// update all items genus properties with new genus id
				for(int j=0;j<buf.size();j++){
					cls[buf[j]].genus_=number_of_genus;
				}
				number_of_genus++;
				break;
			}

			// record current item id
			buf.push_back(idx_tmp); 
			}
		}		
	}

	cout<<cls.size()<<endl;


	// boost::shared_ptr<pcl::visualization::PCLVisualizer> 
	// 	viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	// // Set background
	// // viewer->setBackgroundColor (0.33, 0.97, 0.59); 
	// viewer->setBackgroundColor (1.0f, 1.0f, 1.0f);

	// //Set multi-color for point cloud
	// pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud);  	
	
	// //Add the demostration point cloud data
	// viewer->addPointCloud<PointType> (cloud, multi_color, "cloud1");

	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud1");

	// while(!viewer->wasStopped()){	
	// 	viewer->spin();
	// 	boost::this_thread::sleep (boost::posix_time::microseconds (10));
	// }
	
	return 0;
}
