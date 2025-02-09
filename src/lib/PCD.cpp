#include "PCD.hpp"

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr loadPCD(string filename) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    
    ifstream file(filename);
    if (!file) {
        cerr << "ERROR: Could not open " << filename << "!" << endl;
        return nullptr;
    }

    unordered_map<string, int> field_map;
    int num_points = 0;
    string line;
    vector<string> fields;

    while (getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        stringstream ss(line);
        string token;
        ss >> token;

        if (token == "FIELDS") {
            int idx = 0;
            while (ss >> token) {
                field_map[token] = idx++;
                fields.push_back(token);
            }
        }
        else if (token == "POINTS") {
            ss >> num_points;
        }
        else if (token == "DATA") {
            break;
        }
    }
    
    cloud->width = num_points;
    cloud->height = 1;
    cloud->points.resize(num_points);

    int point_idx = 0;
    while (getline(file, line) && point_idx < num_points) {
        stringstream ss(line);
        vector<float> values(fields.size());
        
        for (int i = 0; i < fields.size(); i++) {
            ss >> values[i];
        }

        auto& point = cloud->points[point_idx];

        if (field_map.count("x")) point.x = values[field_map["x"]];
        if (field_map.count("y")) point.y = values[field_map["y"]];
        if (field_map.count("z")) point.z = values[field_map["z"]];

        if (field_map.count("normal_x")) point.normal_x = values[field_map["normal_x"]];
        if (field_map.count("normal_y")) point.normal_y = values[field_map["normal_y"]];
        if (field_map.count("normal_z")) point.normal_z = values[field_map["normal_z"]];

        if (field_map.count("rgb")) {
            uint32_t rgb_val = *reinterpret_cast<uint32_t*>(&values[field_map["rgb"]]);
            uint8_t r = (rgb_val >> 16) & 0xFF;
            uint8_t g = (rgb_val >> 8) & 0xFF;
            uint8_t b = rgb_val & 0xFF;
            point.r = r;
            point.g = g;
            point.b = b;
        }
                point_idx++;
    }

    file.close();
    return cloud;
}