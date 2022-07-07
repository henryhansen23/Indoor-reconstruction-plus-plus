#include "imu_calls.hpp"

#define HOST "localhost"
#define PORT 4223
#define UID "64tUkb" // Change XXYYZZ to the UID of your IMU Brick 2.0


Imu::Imu()
{
    // Create IP connection
    ipcon_create( &_ipcon );

    // Create device object
    imu_v2_create( &_imu, UID, &_ipcon );
}

Imu::~Imu()
{
    imu_v2_destroy( &_imu );
    ipcon_destroy( &_ipcon ); // Calls ipcon_disconnect internally
}

bool Imu::init( )
{
    // Connect to brickd
    if( ipcon_connect( &_ipcon, HOST, PORT ) < 0 )
    {
        return false;
    }
    return true;
}

bool Imu::get_quaternion( quat_t& v )
{
    int16_t w, x, y, z;

    if( imu_v2_get_quaternion( &_imu, &w, &x, &y, &z ) < 0 )
    {
        return false;
    }
    v = quat_t( w, x, y, z );
    return true;
}

bool Imu::get_gravity_vector( vec3_t& v )
{
    int16_t x, y, z;

    if( imu_v2_get_gravity_vector( &_imu, &x, &y, &z ) < 0 )
    {
        return false;
    }
    v = vec3_t( x, y, z );
    return true;
}

bool Imu::get_rotate_down( quat_t& v )
{
    int16_t roll, yaw, pitch;

    if( imu_v2_get_gravity_vector( &_imu, &roll, &yaw, &pitch ) < 0 )
    {
        return false;
    }
    quat_t r = Eigen::AngleAxisf( roll,  Eigen::Vector3f::UnitX() )
             * Eigen::AngleAxisf( yaw,   Eigen::Vector3f::UnitY() )
             * Eigen::AngleAxisf( pitch, Eigen::Vector3f::UnitZ() );
    r.normalize();
    v = r;
    return true;
}

bool Imu::get_angular_velocity( vec3_t& v )
{
    int16_t x, y, z;

    if( imu_v2_get_angular_velocity( &_imu, &x, &y, &z ) < 0 )
    {
        return false;
    }
    v = vec3_t( x / 16.0f, y / 16.0f, z / 16.0f );
    return true;
}

bool Imu::get_linear_acceleration( vec3_t& v )
{
    int16_t x, y, z;

    if( imu_v2_get_linear_acceleration( &_imu, &x, &y, &z ) < 0 )
    {
        return false;
    }
    v = vec3_t( x / 100.0f, y / 100.0f, z / 100.0f );
    return true;
}

