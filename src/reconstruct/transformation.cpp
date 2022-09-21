#include "transformation.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
static void make_rotation_matrix_from_quaternion( const vec4d_t& quaternion, Eigen::Matrix3d &rotation_matrix )
{
    double qw = quaternion(0), qx = quaternion(1), qy = quaternion(2), qz = quaternion(3);
    rotation_matrix << 1 - 2 * pow(qy, 2) - 2 * pow(qz, 2), 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw,
        2 * qx * qy + 2 * qz * qw, 1 - 2 * pow(qx, 2) - 2 * pow(qz, 2), 2 * qy * qz - 2 * qx * qw,
        2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * pow(qx, 2) - 2 * pow(qy, 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void make_translation_vector(const Eigen::Matrix3d rotation, vec3d_t& tripod_position )
{
    vec3d_t tripod{0, 0, 0.101}; // LiDAR is located 0.101m from tripod joint
    tripod_position = rotation * tripod;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >
make_transformation_matrices( const vec4d_vector_t& quaternions )
{
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > matrices;
    Eigen::Matrix3d tripod_rotation;
    vec3d_t         tripod_position;
    for( std::size_t i = 0; i < quaternions.size(); ++i )
    {
        make_rotation_matrix_from_quaternion( quaternions[i], tripod_rotation );
        make_translation_vector( tripod_rotation, tripod_position );
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        matrix.topLeftCorner<3, 3>() = tripod_rotation;
        matrix.col(3).head<3>() = tripod_position;
        matrices.push_back(matrix);
    }
    return matrices;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
