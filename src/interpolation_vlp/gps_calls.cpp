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
