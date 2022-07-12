#pragma once

#include <vector>
#include <iostream>

#include <pcl/point_cloud.h>

#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

typedef Eigen::Vector3f          vec3_t;
typedef Eigen::Quaternion<float> quat_t;

typedef Eigen::Vector4d          vec4d_t;

typedef std::pair<std::vector<Eigen::Vector4d, 
                              Eigen::aligned_allocator<Eigen::Vector4d> >, 
                  std::vector<double> > quart_vector_t;

typedef std::vector<Eigen::Vector4d,
                    Eigen::aligned_allocator<Eigen::Vector4d> > vector4d_t;

std::ostream& operator<<( std::ostream& ostr, const quat_t& q );

