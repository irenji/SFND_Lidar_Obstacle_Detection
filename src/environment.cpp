/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI> *pointProcessor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
    constexpr float X{ 30.0 }, Y{ 6.5 }, Z{ 2.5 };

    const pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud{ pointProcessor->FilterCloud(
            inputCloud, 0.1f, Eigen::Vector4f(-(15), -6.5, -2.5, 1), Eigen::Vector4f(15, 6.5, 2.5, 1)) };

    renderPointCloud(viewer, filteredCloud, "filteredCloud");

    const std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud{
            pointProcessor->SegmentPlane(filteredCloud, 20, 0.2)
    };

    if (segmentCloud.first->empty() || segmentCloud.second->empty()) { return; }

    renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "Plane Cloud", Color(0, 1, 0));

    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters{ pointProcessor->Clustering(
            segmentCloud.first, 0.35, 100, 1775) };

    if (cloudClusters.empty()) { return; }

    int clusterId{ 0 }, colorIndex{ 0 };

    const std::vector<Color> colors{ Color(1, 0, 1), Color(0, 1, 1), Color(1, 1, 0) };

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(colorIndex));

        Box box(pointProcessor->BoundingBox(cluster));
        renderBox(viewer, box, clusterId);

        ++clusterId;
        ++colorIndex;

        colorIndex %= colors.size();
    }
}

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

    Lidar *lidar = new Lidar(cars, 0.0);
    auto rays = lidar->scan();
    renderPointCloud(viewer, rays, "Rays");

    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    auto result = pointProcessor->SegmentPlane(rays, 100, 0.2);


    renderPointCloud(viewer, result.first, "obstcle", Color(1, 0, 0));
    renderPointCloud(viewer, result.second, "plane", Color(0, 1, 0));
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pp = new ProcessPointClouds<pcl::PointXYZI>();
    auto stream = pp->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIt = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloud = pp->loadPcd(streamIt->string());
        cityBlock(viewer, pp, inputCloud);
        streamIt++;
        if(streamIt == stream.end()) {
            streamIt = stream.begin();
        }

        viewer->spinOnce();
    } 
}