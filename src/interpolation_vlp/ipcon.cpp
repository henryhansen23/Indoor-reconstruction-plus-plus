#include "ipcon.hpp"

#define HOST "localhost"
#define PORT 4223


IPCon::IPCon()
{
    // Create IP connection
    ipcon_create( &_ipcon );
}

IPCon::~IPCon()
{
    ipcon_destroy( &_ipcon ); // Calls ipcon_disconnect internally
}

bool IPCon::init( )
{
    // Connect to brickd
    if( ipcon_connect( &_ipcon, HOST, PORT ) < 0 )
    {
        return false;
    }
    return true;
}

IPConnection& IPCon::getIPCon() {
    return _ipcon;
}
