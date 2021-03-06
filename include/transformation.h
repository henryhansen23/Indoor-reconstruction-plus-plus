#pragma once

#include <vector>

#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

/////////////////////////////////////////////////////////////////////////////////////////////
std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >
make_transformation_matrices(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &quaternions);
/////////////////////////////////////////////////////////////////////////////////////////////
