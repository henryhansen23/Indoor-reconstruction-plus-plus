#pragma once

#include <string>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>

#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

#include "reco_types.h"

///////////////////////////////////////////////////////////////////////////////////////////
int
number_of_scans(const std::string data_path);

int
number_of_directories(const std::string data_path);

std::vector<pcl::PointCloud<pcl::PointXYZ> >
load_fragments( const std::string& data_dir );

std::vector<std::vector<pcl::PointCloud<pcl::PointXYZL> > >
load_datapackets(const std::string path);

void read_quaternions_file( quart_vector_t& quaternions, const std::string path);
///////////////////////////////////////////////////////////////////////////////////////////
