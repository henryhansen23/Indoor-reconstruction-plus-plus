#pragma once

#include <string>

struct Parameters
{
    std::string directory;
    bool        use_odometry{false};
    int         odometry_number;
    float       fov_start;
    float       fov_end;
    bool        wait_gps;
    bool        apply_correction;

    inline void setOdometry( int v )
    {
        use_odometry    = true;
        odometry_number = v;
    }
};

void parseargs( int argc, char** argv, Parameters& params );

