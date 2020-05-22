

#include <Eigen/dense>

using namespace Eigen; 
 

//////////////////////////////////////////////////////////////////////////////////////////////////////////


void make_rotation_matrix_from_quaternion(const Vector4d & quaternion, Matrix3d & rotation_matrix) {

     double qw = quaternion(0), qx = quaternion(1), qy = quaternion(2), qz = quaternion(3); 

     rotation_matrix << 1 - 2 * pow(qy, 2) - 2 * pow(qz, 2), 2 * qx * qy - 2 * qz * qw,                     2 * qx * qz + 2 * qy * qw, 
                        2 * qx * qy + 2 * qz * qw,           1 - 2 * pow(qx, 2) - 2 * pow(qz, 2),           2 * qy * qz - 2 * qx * qw, 
                        2 * qx * qz - 2 * qy * qw,           2 * qy * qz + 2 * qx * qw,           1 - 2 * pow(qx, 2) - 2 * pow(qy, 2); 

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////


void normalize_quaternion(const Vector4d & quaternion, Vector4d & normalized_quaternion) {
    
     double n = sqrt(quaternion.array().pow(2).sum()); 

     normalized_quaternion << quaternion(0) / n, quaternion(1) / n, quaternion(2) / n, quaternion(3) / n; 

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
