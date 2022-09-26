/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* mLidar = new Lidar(cars, 0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr mLidarPCLPtr = mLidar->scan();
    // renderRays(viewer, mLidar->position, mLidarPCLPtr);
    // renderPointCloud(viewer, mLidarPCLPtr, "m_pcl", Color(1,1,1));

    // Point Processor object
    /**Point Processor is a class that processes point clouds*/
    ProcessPointClouds<pcl::PointXYZ>* pProc = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pProc->SegmentPlane(mLidarPCLPtr, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    /*Clustering*/
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pProc->Clustering(segmentCloud.first, 1.0, 3, 30); // cloud-> obstacles
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
        std::cout << "cluster size\n";
        pProc->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pProc->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId++;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


// run scene from pcl stream
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, 
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){

    // filter parameters
    float minX = -10.0f;
    float minY = -5.0f;
    float minZ = -3.0f;

    float maxX = 25.0f;
    float maxY = 8.0f;
    float maxZ = 5.0f;

    float voxelRes = 0.3f;

    // segmentation parameters
    int segMaxIter = 100;
    float segDistanceTh = 0.2;

    // clustering parameters
    float clusterTolerance = 0.6f;
    int minSize = 5;
    int maxSize = 250;

    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud (new pcl::PointCloud<pcl::PointXYZI>);

    /*Filter PCL*/
    filterCloud = pointProcessorI->FilterCloud(inputCloud, voxelRes , Eigen::Vector4f (minX, minY, minZ, 1), Eigen::Vector4f ( maxX, maxY, maxZ, 1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");
    /*Segmentation*/
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->my_SegmentPlane(filterCloud, segMaxIter, segDistanceTh);
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    
    /*Clustering and bounding boxes*/
    /*Clustering*/
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize); // cloud-> obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->my_Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize); // cloud-> obstacles
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){
        std::cout << "cluster size\n";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId++;
    }
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(1,1,1));
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // set up stream
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}