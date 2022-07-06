#pragma once

#include <string>

struct Parameters
{
    std::string directory;
    bool        use_odometry;
    int         odometry_number;
    int         fragment_number;
    float       fov_start;
    float       fov_end;
    bool        apply_correction;

    inline void setOdometry( int v )
    {
        use_odometry    = true;
        odometry_number = v;
    }

    inline void setFragment( int v )
    {
        use_odometry    = false;
        fragment_number = v;
    }
};

void parseargs( int argc, char** argv, Parameters& params );

