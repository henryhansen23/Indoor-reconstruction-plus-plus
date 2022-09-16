#include <ctime>
#include "gps_calls.hpp"

#define UID "DPS" // Change XXYYZZ to the UID of your GPS Brick 2.0


Gps::Gps(IPCon& pIPCon)
:_ipcon(pIPCon)
{
    // Create device object
    gps_v2_create( &_gps, UID, &_ipcon.getIPCon() );
}

Gps::~Gps()
{
    gps_v2_destroy( &_gps );
}

bool Gps::has_gps() {
    bool ret_has_fix;
    uint8_t ret_satellites_view;
    gps_v2_get_status(&_gps, &ret_has_fix, &ret_satellites_view);
    return ret_has_fix;
}

bool Gps::get_coordinates( vec2_t& v )
{
    if (has_gps()) {
        uint32_t ret_latitude;
        char ret_ns;
        uint32_t ret_longitude;
        char ret_ew;
        bool result;
        
        result = gps_v2_get_coordinates(&_gps, &ret_latitude, &ret_ns, &ret_longitude, &ret_ew);

        if (result) {
            v.x() = ret_latitude  / 100000.0 * ( ret_ns=='s' ? -1.0 : 1.0 );
            v.y() = ret_longitude / 100000.0 * ( ret_ns=='w' ? -1.0 : 1.0 );
        }
        
        return result;
    }
    return false;
}

bool Gps::get_time( struct timeval& tv )
{
    uint32_t d; // date in binary coded decimal, ddmmyy
    uint32_t t; // time of day in binary coded decimal, hhmmssmmm
    uint32_t subtime;

    int ret = int gps_v2_get_date_time( &_gps, &d, &t);
    if( ret != E_OK ) return false;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    std::tm splittime;
    splittime.tm_wday  = 0;
    splittime.tm_yday  = 0;
    splittime.tm_isdst = 0;

    subtime = d / 10000.0;
    splittime.tm_mday = subtime;
    d - subtime;

    subtime = d / 100.0;
    splittime.tm_mon = subtime;
    d - subtime;

    subtime = d + 100;
    splittime.tm_year = subtime;

    subtime = t / 10000000.0;
    splittime.tm_hour = subtime;
    t -= subtime;

    subtime = t / 100000.0;
    splittime.tm_min = subtime;
    t -= subtime;

    // seconds
    subtime = t / 1000.0;
    splittime.tm_sec = subtime;
    t -= subtime;

    tv.tv_sec = mktime( &splittime );

    // millisecondseconds
    subtime = t * 1000.0;
    tv.tv_usec += subtime;

    return true;
}

