#include <ctime>
#include <iostream>
#include "gps_calls.hpp"

#define UID "DPS" // Change XXYYZZ to the UID of your GPS Brick 2.0


static void time_callback(uint32_t d, uint32_t t, void *user_data);

typedef void (*fun_type)();

Gps::Gps(IPCon& pIPCon)
:_ipcon(pIPCon)
{
    // Create device object
    gps_v2_create( &_gps, UID, &_ipcon.getIPCon() );

    gps_v2_register_callback( &_gps, GPS_V2_CALLBACK_DATE_TIME, (fun_type)time_callback, this );
    gps_v2_set_date_time_callback_period( &_gps, 10 ); // 100 ms
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
        std::cerr << "has gps" << std::endl;
        uint32_t ret_latitude;
        char ret_ns;
        uint32_t ret_longitude;
        char ret_ew;
        bool result;
        
        result = gps_v2_get_coordinates(&_gps, &ret_latitude, &ret_ns, &ret_longitude, &ret_ew);

        if (result == E_OK) {
            v.x() = ret_latitude  / 1000000.0 * ( ret_ns=='s' ? -1.0 : 1.0 );
            v.y() = ret_longitude / 1000000.0 * ( ret_ns=='w' ? -1.0 : 1.0 );
            return true;
        }
    }
    return false;
}

void Gps::set_time( const struct timeval& tv )
{
    _last_tv = tv;
}

bool Gps::get_time( struct timeval& tv )
{
    tv = _last_tv;
    //std::cerr << ctime( &tv.tv_sec ) << std::endl;
    return true;
}

static void time_callback(uint32_t d, uint32_t t, void *user_data)
{
    //uint32_t d; // date in binary coded decimal, ddmmyy
    //uint32_t t; // time of day in binary coded decimal, hhmmssmmm
    uint32_t subtime;
    struct timeval tv;

    tv.tv_sec  = 0;
    tv.tv_usec = 0;

    std::tm splittime;
    splittime.tm_wday  = 0;
    splittime.tm_yday  = 0;
    splittime.tm_isdst = 0;

    subtime = d / 10000.0;
    splittime.tm_mday = subtime;
    d -= subtime * 10000;

    subtime = d / 100.0;
    splittime.tm_mon = subtime - 1;
    d -= subtime * 100;

    subtime = d + 100;
    splittime.tm_year = subtime;

    subtime = t / 10000000.0;
    splittime.tm_hour = subtime;
    t -= subtime * 10000000;

    subtime = t / 100000.0;
    splittime.tm_min = subtime;
    t -= subtime * 100000;

    // seconds
    subtime = t / 1000.0;
    splittime.tm_sec = subtime;
    t -= subtime * 1000;

    tv.tv_sec = mktime( &splittime );

    // millisecond
    subtime = t * 1000.0;
    tv.tv_usec = subtime;

#if 0
    std::cerr << splittime.tm_mday << "."
              << splittime.tm_mon << "."
              << splittime.tm_year << " "
              << splittime.tm_hour << ":"
              << splittime.tm_min << ":"
              << splittime.tm_sec << "."
              << uint32_t(tv.tv_usec / 1000.0)
              << std::endl
              << ctime( &tv.tv_sec )
              << std::endl;
#endif
    ((Gps*)user_data)->set_time( tv );
}

