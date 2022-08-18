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
