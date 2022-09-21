#include "transformation.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
static void make_rotation_matrix_from_quaternion( const vec4d_t& quaternion, mat3d_t& rotation_matrix )
{
    double qw = quaternion(0), qx = quaternion(1), qy = quaternion(2), qz = quaternion(3);
    rotation_matrix << 1 - 2 * pow(qy, 2) - 2 * pow(qz, 2), 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw,
        2 * qx * qy + 2 * qz * qw, 1 - 2 * pow(qx, 2) - 2 * pow(qz, 2), 2 * qy * qz - 2 * qx * qw,
        2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * pow(qx, 2) - 2 * pow(qy, 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void make_translation_vector(const mat3d_t& rotation, vec3d_t& tripod_position )
{
    vec3d_t tripod{0, 0, 0.101}; // LiDAR is located 0.101m from tripod joint
    tripod_position = rotation * tripod;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
mat4d_vector_t make_transformation_matrices( const vec4d_vector_t& quaternions )
{
    mat4d_vector_t  matrices;
    mat3d_t         tripod_rotation;
    vec3d_t         tripod_position;
    for( std::size_t i = 0; i < quaternions.size(); ++i )
    {
        make_rotation_matrix_from_quaternion( quaternions[i], tripod_rotation );
        make_translation_vector( tripod_rotation, tripod_position );

        mat4d_t matrix = mat4d_t::Identity();
        matrix.topLeftCorner<3, 3>() = tripod_rotation;
        matrix.col(3).head<3>() = tripod_position;
        matrices.push_back(matrix);
    }
    return matrices;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

