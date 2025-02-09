#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <unordered_map>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr loadPCD(string filename);