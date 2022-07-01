//
// Created by Branislav Jenco on 21.09.2020.
// Crop point cloud with minX/Y/Z and maxX/Y/Z boundaries, fit a plane and output a file with signed
// distances between the fitted plane and points.
// Assuming the cropped point cloud is of a flat surface to which a plane can be fitted.
// The input must be a PointXYZL point cloud (with labels)
//

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <iostream>
#include <fstream>

typedef pcl::PointCloud<pcl::PointXYZL> point_cloud;

int
main(int argc, char **argv)
{
    point_cloud::Ptr cloud (new point_cloud);
    point_cloud::Ptr cropped (new point_cloud);

    std::string input_file;
    std::string output_file;

    std::vector<float> boundaries(6);
    pcl::console::parse_x_arguments(argc, argv, "-b", boundaries);

    char opt_f[] = "-f";
    char opt_o[] = "-o";
    pcl::console::parse_argument(argc, (char**)argv, opt_f, input_file);
    pcl::console::parse_argument(argc, (char**)argv, opt_o, output_file);
    for (auto const& v : boundaries)
    {
        std::cout << v << " " << std::flush;
    }
    std::cout << std::endl;

    float minX = boundaries[0], minY = boundaries[2], minZ = boundaries[4];
    float maxX = boundaries[1], maxY = boundaries[3], maxZ = boundaries[5];


    std::cout << "Loading file" << std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZL>(input_file, *cloud);

    pcl::CropBox<pcl::PointXYZL> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cropped);

    pcl::SampleConsensusModelPlane<pcl::PointXYZL>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZL> (cropped));
    pcl::RandomSampleConsensus<pcl::PointXYZL> ransac (model_p);
    ransac.setDistanceThreshold (.01);

    std::cout << "Computing plane" << std::endl;
    ransac.computeModel();
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);

    struct DistanceData {
        double distance_signed;
        int laser_id;
        float* data;
    };

    std::vector<DistanceData> dists;

    for (auto & it : *cropped)
    {
        DistanceData d{};
        d.distance_signed = pcl::pointToPlaneDistanceSigned(it, coeff);
        d.laser_id = it.label;
        d.data = it.data;
        dists.emplace_back(d);
    }

    ofstream myfile;
    myfile.open(output_file);

    for (auto & dist : dists)
    {
        myfile << dist.distance_signed << " " << dist.data[0] << " " << dist.data[1] << " " << dist.data[2] << " " << dist.data[3] << " " << dist.laser_id << std::endl;
    }
    myfile.close();

    pcl::visualization::PCLVisualizer viz;
    viz.setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZL> color(cropped);
    viz.addPointCloud<pcl::PointXYZL>(cropped, color, "wall");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "wall");
    viz.spin();
}