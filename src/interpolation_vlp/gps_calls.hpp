#pragma once

// Tinkerforge GPS2.0
#include "Tinkerforge/bricklet_gps_v2.h"

#include "ipcon.hpp"

// Eigen3
#include <eigen3/Eigen/Geometry>

/*
 * The Gps class hides the Tinkerforge GPS 2.0. If other GPSs
 * are supported later, this can probably be subclassed.
 */
class Gps
{
    IPCon& _ipcon;
    GPSV2        _gps;
public:
    Gps(IPCon& pIPCon);
    ~Gps();
};

