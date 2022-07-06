#include <iostream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <sys/types.h>
#include <ifaddrs.h> // for getifaddrs
#include <netinet/in.h> // for sockaddr_in

// VelodyneCapture
#include "VelodyneCapture.h"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

// Boost
#include <boost/filesystem.hpp>

// Tinkerforge IMU2.0
#include "Tinkerforge_IMU2.0/ip_connection.h"
#include "Tinkerforge_IMU2.0/brick_imu_v2.h"

// eigen3
#include <eigen3/Eigen/Geometry>

// cmath
#include <cmath>

#define HOST "localhost"
#define PORT 4223
#define UID "64tUkb" // Change XXYYZZ to the UID of your IMU Brick 2.0
#define PI 3.14159265359

#define VLP_ADDRESS "192.168.1.201"
#define VLP_PORT    2368

volatile sig_atomic_t interrupted = false;

void signal_handler(int s)
{
    interrupted = true;
}

double clamp(double v)
{
    const double t = v < 0 ? 0 : v;
    return t > 1.0 ? 1.0 : t;
}

const std::vector<float> vertical_correction = { 11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7, -11.2 };


static void usage( const char* prog_name );

static bool validate_interface( const char* address );

///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// Main ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////


int main( int argc, const char *const *argv )
{
    signal(SIGINT, signal_handler);

    // -------------------------------------------
    // ------ COMMAND-LINE ARGUMENT PARSING ------
    // -------------------------------------------
    
    int laser_num;
    int fov_start;
    int fov_end;
    bool write_pcd;
    bool apply_correction;
    std::vector <std::string> pcds;

    if( validate_interface( VLP_ADDRESS ) == false )
    {
        cerr << "This computer has no network interface configured for reception from the VLP-16" << endl;
        return 1;
    }

// ------------------------------------
// ------ SET UP IMU CONNECTION -------
// ------------------------------------

    // Create IP connection
    IPConnection ipcon{};
    ipcon_create(&ipcon);

    // Create device object
    IMUV2 imu{};
    imu_v2_create(&imu, UID, &ipcon);

    // Connect to brickd
    if (ipcon_connect(&ipcon, HOST, PORT) < 0) {
        std::cerr << "Could not connect\n";
        return 1; // Don't use device before ipcon is connected
    }

// -------------------------------------------------

    // Initialize pcl point cloud with point type -> used for pcd file
    pcl::PointCloud<pcl::PointXYZRGBL> cloud;

    // Initialize some loop variables
    int number = 0; // Counter for number of pcds in one 360 degree frame
    int frame_count = 0; // Counter for a full 360 degree rotation
    double azimuth_rotation = fov_end; // Used to check n-1 azimuth rotation
    long timestamp = 0;

    // ----------------------------------------------------------------
    // --------------------SET UP VLP CONNECTION ----------------------
    // ----------------------------------------------------------------

    // Connect to ipadress and port
    const boost::asio::ip::address ipaddress = boost::asio::ip::address::from_string( VLP_ADDRESS );
    const unsigned short port = VLP_PORT;
    velodyne::VLP16Capture capture(ipaddress, port);

    // Check if capture is open
    if (!capture.isOpen()) {
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        usage(argv[0]);
        return -1;
    }
    
    // Print
    else {
        std::cout << "Capture from Sensor..." << std::endl;
        std::cout << "ipadress : " << ipaddress << std::endl;
        std::cout << "port : " << port << std::endl;
        std::cout << "\n\n";
    }

    //--------------------
    // ---- Main loop ----
    //--------------------

    bool quaternionsInitialized = false;
    Eigen::Quaternion<float> startQuaternion;
    Eigen::Quaternion<float> endQuaternion;

    int ct = 1000;
    while (capture.isRun() && !interrupted) {
        ct--;
        if( ct<0 ) { cerr << "."; ct = 1000; }

        // Get quaternions from imu
        int16_t q_w, q_x, q_y, q_z;

        if (imu_v2_get_quaternion(&imu, &q_w, &q_x, &q_y, &q_z) < 0) {
          std::cerr << "Could not get quaternion, probably timeout" << "\n";
        }

        // Get gravity from imu
        int16_t g_x, g_y, g_z;

        if (imu_v2_get_gravity_vector(&imu, &g_x, &g_y, &g_z) < 0) {
          std::cerr << "Could not get gravity vector, probably timeout" << "\n";
        }
        
        Eigen::Quaternion<float> tempQuaternion = Eigen::Quaternion<float>(q_w, q_x, q_y, q_z);
        tempQuaternion.normalize();
        
        if (!quaternionsInitialized) {
            quaternionsInitialized = true;
            startQuaternion = tempQuaternion;
            endQuaternion = tempQuaternion;
        } else {
            if (tempQuaternion.angularDistance(startQuaternion) > endQuaternion.angularDistance(startQuaternion)) {
                endQuaternion = tempQuaternion;
            } else if (endQuaternion.angularDistance(tempQuaternion) > endQuaternion.angularDistance(startQuaternion)) {
                startQuaternion = tempQuaternion;
            }
            Eigen::Quaternion<float> difference = (startQuaternion.conjugate() * endQuaternion);

            std::cout << "Current angular distance: " << endQuaternion.angularDistance(startQuaternion) * 180 / M_PI << "\n";
            std::cout << "Closeness to full rotation along X axis: " << difference.toRotationMatrix()(0, 0) << "\n";
            std::cout << "Angle distance to full rotation along X axis: " << roundf(acos(difference.toRotationMatrix()(0, 0)) * 180 / M_PI * 10) / 10 << " degree" << "\n";
        }

        // Get angular velocity from imu
        int16_t angv_x, angv_y, angv_z;
        if (imu_v2_get_angular_velocity(&imu, &angv_x, &angv_y, &angv_z) < 0) {
          std::cerr << "Could not get angular velocity, probably timeout" << "\n";
        }

        // Get linear acceleration from imu
        int16_t lina_x, lina_y, lina_z;
        if (imu_v2_get_linear_acceleration(&imu, &lina_x, &lina_y, &lina_z) < 0) {
          std::cerr << "Could not get linear acceleration, probably timeout" << "\n";
        }

        frame_count++;
    }

    imu_v2_destroy(&imu);
    ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally
    return 0;
}

static void usage( const char* prog_name )
{
     std::cout << "\n\nUsage: "<<prog_name<<" [options]\n\n"
                  "This program calibrates the IMU2.0 sensor\n\n"
                  "Options:\n"
                  "-------------------------------------------------------------------------------\n"
                  "-h             This help message\n"
                  "-------------------------------------------------------------------------------\n"
                  "\n\n";
}

static bool validate_interface( const char* address )
{
    struct in_addr vlp_addr;
    int err = inet_pton( AF_INET, address, &vlp_addr );
    if( err == 0 )
    {
        perror( "No valid IPv4 address for VLP given. " );
        return false;
    }

    struct ifaddrs *ifap;
    err = getifaddrs( &ifap );
    if( err != 0 )
    {
        perror( "Cannot get list of network interfaces. " );
        return false;
    }

    for( struct ifaddrs* it = ifap; it != nullptr; it=it->ifa_next )
    {
        if( it->ifa_addr->sa_family != AF_INET ) continue;

        uint32_t net;
        net  = ((struct sockaddr_in*)it->ifa_addr   )->sin_addr.s_addr;
        net &= ((struct sockaddr_in*)it->ifa_netmask)->sin_addr.s_addr;

        if( ( net & vlp_addr.s_addr ) == net )
        {
            cout << "Found a good network interface " << it->ifa_name << endl;
            freeifaddrs ( ifap );
            return true;
        }
    }
    cout << "No good network interface for talking to " << VLP_ADDRESS << " found." << endl
         << "Check your network." << endl;
    freeifaddrs ( ifap );
    return false;
}

