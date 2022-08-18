#pragma once

// Tinkerforge IMU2.0
#include "Tinkerforge/brick_imu_v2.h"

#include "ipcon.hpp"

// Eigen3
#include <eigen3/Eigen/Geometry>

/*
 * quat_t as a shorthand for using quaternions
 */
typedef Eigen::Quaternion<float> quat_t;

/*
 * vec3_t as a shorthand for using Eigen::Vector3f
 */
typedef Eigen::Vector3f vec3_t;

/*
 * The Imu class hides the Tinkerforge IMU. If other IMUs
 * are supported later, this can probably be subclassed.
 */
class Imu
{
    IPCon&     _ipcon;
    IMUV2        _imu;
public:
    Imu(IPCon& pIPCon);
    ~Imu();

    bool get_quaternion( quat_t& v );
    bool get_gravity_vector( vec3_t& v );

    /** Read the gravity vector, and it as a normalized rotation of the current
     *  Lidar coordinate system.
     */
    bool get_rotate_down( quat_t& v );

    /** The float vector contains the angular velocities in degrees per second.
     *  Converted from IMU 2 which returns int16_t as multiples of a 16th of a degree per second.
     */
    bool get_angular_velocity( vec3_t& v );

    /** The float vector contains the linear acceleration in meters per second.
     *  Converted from IMU 2, which returns in16_t as multiples of centimeters per second.
     */
    bool get_linear_acceleration( vec3_t& v );
};

