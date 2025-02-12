#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    // Ensure we have a file to read
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path-to-pcd-file>\n";
        return -1;
    }

    // Create a container for the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Load the point cloud from a PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file. Make sure the file path is correct.\n");
        return -1;
    }
    std::cout << "Loaded " << cloud->width * cloud->height
              << " data points from " << argv[1] << std::endl;

    // Create a PCLVisualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // black background

    // Add the point cloud to the viewer
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "sample cloud"); 

    // Keep the viewer open until the user closes it
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
