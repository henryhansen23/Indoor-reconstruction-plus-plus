#pragma once

#include <vector>

#include <pcl/point_cloud.h>

#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

typedef std::pair<std::vector<Eigen::Vector4d, 
                              Eigen::aligned_allocator<Eigen::Vector4d> >, 
                  std::vector<double> > quart_vector_t;

typedef std::vector<Eigen::Vector4d,
                    Eigen::aligned_allocator<Eigen::Vector4d> > vector4d_t;

