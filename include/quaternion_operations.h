

#pragma once

 
#include <Eigen/dense>

using namespace Eigen; 


/////////////////////////////////////////////////////////////////////////////


void make_rotation_matrix_from_quaternion(const Vector4d & quaternion, Matrix3d & rotation_matrix); 


void normalize_quaternion(const Vector4d & quaternion, Vector4d & normalized_quaternion); 


/////////////////////////////////////////////////////////////////////////////
