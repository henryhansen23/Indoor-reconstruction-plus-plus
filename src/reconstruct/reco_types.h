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
typedef Eigen::Vector3d          vec3d_t;
typedef Eigen::Vector4d          vec4d_t;
typedef Eigen::Matrix3d          mat3d_t;
typedef Eigen::Matrix4f          mat4f_t;
typedef Eigen::Matrix4d          mat4d_t;
typedef Eigen::Quaternion<float> quat_t;

typedef std::vector<vec3_t,  Eigen::aligned_allocator<vec3_t > > vec3_vector_t;
typedef std::vector<vec4d_t, Eigen::aligned_allocator<vec4d_t> > vec4d_vector_t;
typedef std::vector<mat4d_t, Eigen::aligned_allocator<mat4d_t> > mat4d_vector_t;

typedef std::pair<vec4d_vector_t,
                  std::vector<double> > quart_vector_t;

std::ostream& operator<<( std::ostream& ostr, const quat_t& q );

