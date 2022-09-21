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
void translation_estimation( const std::string& path, vec3_t& translation );

void incremental_pairwise_registration( const std::vector<pcl::PointCloud<pcl::PointXYZ> > &clouds,
                                        const vec3_vector_t& translations,
                                        const std::string&   combined_file,
                                        const std::string&   icp_type,
                                        const bool           visualization );
//////////////////////////////////////////////////////////////
