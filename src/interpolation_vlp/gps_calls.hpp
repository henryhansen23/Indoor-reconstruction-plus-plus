#pragma once

// Tinkerforge GPS2.0
#include "Tinkerforge/bricklet_gps_v2.h"

#include "ipcon.hpp"

// Eigen3
#include <eigen3/Eigen/Geometry>

typedef Eigen::Vector2f vec2_t;

/*
 * The Gps class hides the Tinkerforge GPS 2.0. If other GPSs
 * are supported later, this can probably be subclassed.
 */
class Gps
{
    IPCon& _ipcon;
    GPSV2        _gps;
    struct timeval _last_tv;
public:
    Gps(IPCon& pIPCon);
    ~Gps();
    
    bool has_gps();
    bool get_coordinates( vec2_t& v );

    bool get_time( struct timeval& tv );

    void set_time( const struct timeval& tv );
};

