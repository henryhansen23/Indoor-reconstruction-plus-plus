

#pragma once

 
#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


/////////////////////////////////////////////////////////////////////////////


void make_rotation_matrix_from_quaternion(const Eigen::Vector4d & quaternion, Eigen::Matrix3d & rotation_matrix); 


void normalize_quaternion(const Eigen::Vector4d & quaternion, Eigen::Vector4d & normalized_quaternion); 


/////////////////////////////////////////////////////////////////////////////
