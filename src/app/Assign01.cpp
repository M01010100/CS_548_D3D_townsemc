#include "PCD.hpp"
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Error: Please provide input file path" << endl;
        return 1;
    }

    auto cloud = loadPCD(string(argv[1]));
    if (!cloud) {
        return 1;
    }

    auto viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    viewer->setBackgroundColor(0.7, 0.7, 0.7);
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud, "cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud, 10, 0.01, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "normals");
    
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return 0;
}