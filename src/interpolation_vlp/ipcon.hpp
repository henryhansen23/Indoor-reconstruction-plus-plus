#pragma once

#include "Tinkerforge/ip_connection.h"

class IPCon
{
    IPConnection _ipcon;
public:
    IPCon();
    ~IPCon();

    IPConnection& getIPCon();

    bool init( );
};

