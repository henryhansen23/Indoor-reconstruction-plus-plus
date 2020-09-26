#include <string>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

#include <boost/filesystem.hpp>

#include "load_data.h"

using namespace boost::filesystem;

///////////////////////////////////////////////////////////////////////////////////////////
int
number_of_scans(const std::string data_path)
{
    constexpr int pos_start_number = 5;
    int number = 0;
    path p(data_path);
    for (auto i = directory_iterator(p); i != directory_iterator(); ++i) {
        std::string file = i->path().filename().string();
        if (file == ".DS_Store") { continue; } // If Apple

        int new_number = std::stoi(file.substr(pos_start_number, file.find("_", pos_start_number) - pos_start_number));
        if (new_number > number) {
            number = new_number;
        }
    }
    return number + 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
int
number_of_directories(const std::string data_path)
{
    int number = 0;
    path p(data_path);
    for (auto i = directory_iterator(p); i != directory_iterator(); ++i) {
        std::string file = i->path().filename().string();
        if (file == ".DS_Store") { continue; } // If Apple
        int new_number = std::stoi(file.substr(file.find("_") + 1, file.length()));
        if (new_number > number) {
            number = new_number;
        }
    }
    return number + 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<pcl::PointCloud<pcl::PointXYZ> >
load_fragments(const std::string fragments_path, const int fragments_number)
{
    // Read clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
    for (int i = 0; i < fragments_number; ++i) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::io::loadPCDFile<pcl::PointXYZ>(fragments_path + "/fragment_" + std::to_string(i) + "/fragment.pcd", cloud);
        clouds.push_back(cloud);
    }
    return clouds;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<pcl::PointCloud<pcl::PointXYZ> >
load_datapackets_in_one_scan(const int scan_number, const std::string datapackets_path)
{
    constexpr int pos_scan_number = 5;
    path p(datapackets_path);
    std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
    for (auto i = directory_iterator(p); i != directory_iterator(); ++i) {
        std::string file = i->path().filename().string();
        if (file == ".DS_Store") { continue; } // If Apple
        std::string scan_file_number = file.substr(pos_scan_number, file.find("_", pos_scan_number) - pos_scan_number);
        if (scan_number == std::stoi(scan_file_number)) {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::io::loadPCDFile<pcl::PointXYZ>(datapackets_path + "/" + file, cloud);
            clouds.push_back(cloud);
        }
    }
    return clouds;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>>>
load_datapackets(const std::string path)
{
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ> > > fragment_clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZ> > datapackets_scan_clouds;
    int n_scans = number_of_scans(path);
    for (int i = 0; i < n_scans; ++i) {
        datapackets_scan_clouds = load_datapackets_in_one_scan(i, path);
        fragment_clouds.push_back(datapackets_scan_clouds);
    }
    return fragment_clouds;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_quaternions_file(quart_vector_t& quaternions, const std::string path)
{
    std::ifstream input(path + "/" + "quaternions_datapacket.csv");
    const std::string delimiter = ",";
    std::string line;
    // Read lines
    std::getline(input, line); // Read first line
    while (std::getline(input, line)) {
        int pos = 0;
        std::vector<double> row;
        // Split numbers by delimiter
        while ((pos = line.find(delimiter)) != std::string::npos) {
            double number = std::stod(line.substr(0, pos));
            row.push_back(number);
            line.erase(0, pos + delimiter.length());
        }
        // Store row
        Eigen::Vector4d quat;
        for (int j = 0; j < 4; ++j) {
            quat(j) = row[j];
        }
        double time = row[4];
        quaternions.first.push_back(quat);
        quaternions.second.push_back(time);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


