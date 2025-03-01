#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;
using namespace pcl;

int main(int argc, char **argv) {
    PointCloud<PointXYZRGB>::Ptr cloud(
        new PointCloud<PointXYZRGB>());

    if(pcl::io::loadPCDFile<PointXYZRGB>(
        "data/assign01/BunnyXYZ.pcd", *cloud) == -1) {
        cerr << "CANNOT LOAD FILE!" << endl;
        return 1;
    }

    PointXYZ minP;
    PointXYZ maxP;
    for(int i = 0; i < cloud->points.size(); i++) {
        cloud->points.at(i).r = 255;
        cloud->points.at(i).g = 255;
        cloud->points.at(i).b = 0;

        minP.x = min(minP.x, cloud->points.at(i).x);
        minP.y = min(minP.y, cloud->points.at(i).y);
        minP.z = min(minP.z, cloud->points.at(i).z);

        maxP.x = max(maxP.x, cloud->points.at(i).x);
        maxP.y = max(maxP.y, cloud->points.at(i).y);
        maxP.z = max(maxP.z, cloud->points.at(i).z);
    }

    cout << minP << endl;
    cout << maxP << endl;
    PointXYZ dimP;
    dimP.x = maxP.x - minP.x;
    dimP.y = maxP.y - minP.y;
    dimP.z = maxP.z - minP.z;
    cout << "DIMENSIONS: " << dimP << endl;

    auto startTime = chrono::steady_clock().now();

    /*
    float resolution = 0.001f;
    pcl::octree::OctreePointCloudSearch<PointXYZRGB> tree(resolution);
    tree.setInputCloud(cloud);
    tree.addPointsFromInputCloud();
    */

   

   pcl::KdTreeFLANN<PointXYZRGB> tree;
   tree.setInputCloud(cloud);

    vector<int> indices;
    vector<float> dist;
    float radius = 0.02f;
    if(tree.radiusSearch(700, radius, indices,
                            dist) > 0) {
        for(int i = 0; i < indices.size(); i++) {
            int index = indices.at(i);
            cloud->points.at(index).b = 255;
        }
                            }                      
    
    auto endTime = chrono::steady_clock().now();
    auto totalTime = endTime - startTime;
    //cout << "TIME: " << totalTime << endl;


    // Create a PCLVisualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // black background

    // Add the point cloud to the viewer
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "bunny");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "bunny");
    
    // Keep the viewer open until the user closes it
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }   

    return 0;
}