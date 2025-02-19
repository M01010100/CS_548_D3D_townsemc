#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>
#include "PCD.hpp"
using namespace std;

bool is_point_near_equal(pcl::PointXYZRGBNormal p1, pcl::PointXYZRGBNormal p2, float eps) {
    return ((fabs(p1.x - p2.x) <= eps) &&
            (fabs(p1.y - p2.y) <= eps) &&
            (fabs(p1.z - p2.z) <= eps) &&
            (fabs(p1.normal_x - p2.normal_x) <= eps) &&
            (fabs(p1.normal_y - p2.normal_y) <= eps) &&
            (fabs(p1.normal_z - p2.normal_z) <= eps) &&
            (fabs(p1.r - p2.r) <= eps) &&
            (fabs(p1.g - p2.g) <= eps) &&
            (fabs(p1.b - p2.b) <= eps));

}

void test_one_file(string filename) {
    // Load data with user function
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcd = loadPCD(filename);
    // Load data with PCL function
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ground_pcd(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(filename, *ground_pcd) == -1) {
        cerr << "TEST PROGRAM ERROR: Could not load PCD file: " << filename << endl;
        throw std::exception();
    }

    // Check width and height
    CHECK(pcd->width == ground_pcd->width);
    CHECK(pcd->height == ground_pcd->height);

    // Get point data
    auto points = pcd->points;
    auto ground_points = ground_pcd->points;

    // Do we have the same number of points?
    CHECK(points.size() == ground_points.size());

    // Do our points match?
    for(int i = 0; i < points.size(); i++) {
        auto p = points.at(i);
        auto gp = ground_points.at(i);
        CHECK(is_point_near_equal(p, gp, 1e-5));
    }
}

TEST_CASE("Non-existent file check") {
    // Test bad file
    CHECK(loadPCD("NOT_HERE.pcd") == nullptr);
}

// Test actual data
TEST_CASE("Loading BunnyXYZ.pcd") {
    test_one_file("./data/assign01/BunnyXYZ.pcd");  
}

TEST_CASE("Loading BunnyXYZN.pcd") {
    test_one_file("./data/assign01/BunnyXYZN.pcd"); 
}

TEST_CASE("Loading BunnyXYZNRGB.pcd") {
    test_one_file("./data/assign01/BunnyXYZNRGB.pcd");
}

TEST_CASE("Loading BunnyXYZRGB.pcd") {
    test_one_file("./data/assign01/BunnyXYZRGB.pcd");    
}
