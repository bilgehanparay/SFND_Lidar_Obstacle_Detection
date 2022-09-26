/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <iostream>
#include "../../processPointClouds.h"

// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// keep randomly selected points
	std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> randPoints;
	std::vector<int> inlierCount;
	// For max iterations 
	for(int i=0; i<maxIterations; i++){
		// Randomly sample subset and fit line
		/*register the picked points*/
		randPoints.push_back(std::make_pair(cloud->points[rand() % cloud->points.size()], 
								       	    cloud->points[rand() % cloud->points.size()]) );
		/*Compute line parameters*/
		float x1 = randPoints[i].first.x;
		float y1 = randPoints[i].first.y;

		float x2 = randPoints[i].second.x;
		float y2 = randPoints[i].second.y;
		float A = y1-y2; 
		float B = x2-x1;
		float C = x1*y2 - x2*y1;
		// Measure distance between every point and fitted line
		int inCountThisLine = 0;
		for(int j = 0; j < cloud->points.size(); j++){
			float x = cloud->points[j].x;
			float y = cloud->points[j].y;
			float d = fabs(A*x + B*y + C) / sqrt(A*A + B*B);
			// If distance is smaller than threshold count it as inlier	
			if(d <= distanceTol)
				inCountThisLine++;
		}
		inlierCount.push_back(inCountThisLine);
	}
	// Return indicies of inliers from fitted line with most inliers
	int maxInlierIx = std::distance(inlierCount.begin(), 
									max_element(inlierCount.begin(), inlierCount.end()));
	float x1 = randPoints[maxInlierIx].first.x;
	float y1 = randPoints[maxInlierIx].first.y;

	float x2 = randPoints[maxInlierIx].second.x;
	float y2 = randPoints[maxInlierIx].second.y;
	float A = y1-y2; 
	float B = x2-x1;
	float C = x1*y2 - x2*y1;
	for(int j = 0; j < cloud->points.size(); j++){
		float x = cloud->points[j].x;
		float y = cloud->points[j].y;
		float d = fabs(A*x + B*y + C) / sqrt(A*A + B*B);
		// If distance is smaller than threshold count it as inlier	
		if(d <= distanceTol)
			inliersResult.insert(j);
	}
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIte, float disTo){
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	
	for(int ix=0; ix< maxIte; ix++){
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand() % cloud->points.size());
		
		auto itr = inliers.begin();
		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		float z1 = cloud->points[*itr].z;
		itr++;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;
		float z2 = cloud->points[*itr].z;
		itr++;
		float x3 = cloud->points[*itr].x;
		float y3 = cloud->points[*itr].y;
		float z3 = cloud->points[*itr].z;
		std::vector<float> v1 {x2-x1, y2-y1, z2-z1};
		std::vector<float> v2 {x3-x1, y3-y1, z3-z1};
		float i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1); // i
		float j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1); // j
		float k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1); // k
		float A = i;
		float B = j;
		float C = k;
		float D = -(i*x1 + j*y1 + k*z1);
		// Ax + By + Cz + D = 0 => eqn. of plane
		for(int ij = 0; ij < cloud->points.size(); ij++){
			if(inliers.count(ij) > 0) // dont add points already added)
				continue;
			float x = cloud->points[ij].x;
			float y = cloud->points[ij].y;
			float z = cloud->points[ij].z;
			float d = fabs(A*x + B*y + C*z + D) / sqrt(A*A + B*B + C*C);
			if(d < disTo){
				inliers.insert(ij);
			}
		}
		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}
	}
	return inliersResult;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac2D(cloud, 50, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
