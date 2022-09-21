#pragma once

#include <string>
#include <vector>

#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

#include <pcl/point_cloud.h>

///////////////////////////////////////////////////////////////
void
translation_estimation(const std::string path, Eigen::Vector3f &translation);

void
incremental_pairwise_registration(const std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds,
                                  const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &translations,
                                  const std::string& combined_file,
                                  const std::string& icp_type,
                                  const bool visualization);
//////////////////////////////////////////////////////////////
