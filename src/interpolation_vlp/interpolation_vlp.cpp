#include <iostream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <sys/types.h>
#include <ifaddrs.h> // for getifaddrs
#include <netinet/in.h> // for sockaddr_in

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

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

#include "cmdline.hpp"
#include "imu_calls.hpp"
#include "gps_calls.hpp"

// #define HOST "localhost"
// #define PORT 4223
// #define UID "64tUkb" // Change XXYYZZ to the UID of your IMU Brick 2.0
#define PI 3.14159265359

#define VLP_ADDRESS "192.168.1.201"
#define VLP_PORT    2368

volatile sig_atomic_t interrupted = false;

void signal_handler(int s)
{
    interrupted = true;
}

inline double clamp(double v)
{
    // const double t = v < 0 ? 0 : v;
    // return t > 1.0 ? 1.0 : t;
    return std::min( std::max( v, 0.0 ), 1.0 );
}

const std::vector<float> vertical_correction = { 11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7, -11.2 };


static bool validate_interface( const char* address );

///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// Main ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////


int main( int argc, char **argv )
{
    signal(SIGINT, signal_handler);

    // -------------------------------------------
    // ------ COMMAND-LINE ARGUMENT PARSING ------
    // -------------------------------------------

    Parameters params;

    parseargs( argc, argv, params );

    // std::string directory;
    // std::string fragment;
    // std::string odometry;
    int laser_num;
    // int fov_start;
    // int fov_end;
    bool write_pcd;
    // bool apply_correction;
    std::vector <std::string> pcds;

    if( validate_interface( VLP_ADDRESS ) == false )
    {
        cerr << "This computer has no network interface configured for reception from the VLP-16" << endl;
        return 1;
    }

// ------------------------------------
// ------ SET UP IMU CONNECTION -------
// ------------------------------------

    IPCon ipcon;
    ipcon.init( );

    Imu imu(ipcon);
    Gps gps(ipcon);
    
    if (params.wait_gps) {
        while (!gps.has_gps()) {
            std::cout << "Waiting for GPS fix\n";
            sleep(1);
        }
    }

// -------------------------------------------------

    std::string path;
    if( params.use_odometry )
    {
        path = params.directory + "/odometry/odometry_" + std::to_string(params.odometry_number) + "/";
        std::cout << "Writing to: odometry_" << params.odometry_number << "\n\n";
    }
    else
    {
        path = params.directory + "/fragments/fragment_" + std::to_string(params.fragment_number) + "/";
        std::cout << "Writing to: fragment_" << params.fragment_number << "\n\n";
    }

    // Create directories in path
    cout << "Creating the path " << path << endl;
    boost::filesystem::create_directories(path);

    // Open and write header to quaternion csv file
    boost::filesystem::create_directories(path + "/quaternions");
    std::ofstream quaternion;
    quaternion.open(path + "/quaternions/quaternions_datapacket.csv");
    quaternion << "q_w,q_x,q_y,q_z, g_x,g_y,g_z, rot_w,rot_x,rot_y,rot_z, t, c\n";
    quaternion.close();

    // Open and write header to imu data csv file
    boost::filesystem::create_directories(path + "/imu");
    std::ofstream imu_data;
    imu_data.open(path + "/imu/imu_data.csv");
    imu_data << "angv_x,angv_y,angv_z,lina_x,lina_y,lina_z\n";
    imu_data.close();

    // Open and write header to location csv file
    boost::filesystem::create_directories(path + "/location");
    std::ofstream location;
    location.open(path + "/location/location_datapacket.csv");
    location << "time,lon,lat\n";
    location.close();


    // Initialize some loop variables
    int number = 0; // Counter for number of pcds in one 360 degree frame
    int frame_count = 0; // Counter for a full 360 degree rotation
    double azimuth_rotation = params.fov_end; // Used to check n-1 azimuth rotation
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
        return -1;
    }
    
    // Print
    else {
        std::cout << "Capture from Sensor..." << std::endl;
        std::cout << "ipadress : " << ipaddress << std::endl;
        std::cout << "port : " << port << std::endl;
        std::cout << "\n\n";
    }

    Eigen::Quaternion<float> startQuaternion;
    bool startQuaternionSet = false;

    //--------------------
    // ---- Main loop ----
    //--------------------

    int ct = 1000;
    while (capture.isRun() && !interrupted)
    {
        ct--;
        if( ct<=0 ) { cerr << "."; ct = 1000; }

        // Initialize lasers with velodyne data
        std::vector <velodyne::Laser> lasers;
        capture >> lasers;
        if (lasers.empty()) {
            continue;
        }

        // Initialize pcl point cloud with point type -> used for pcd file
        pcl::PointCloud<pcl::PointXYZRGBL> cloud;

        // Fill in the cloud data -> used for writing to pcd file
        cloud.width = static_cast <uint32_t>(lasers.size());
        cloud.height = 1;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);

        // Bool used to write pcds if pcd is inside given fov scope
        write_pcd = true;

        // Initialize counter
        int j = 0;

        // rotation towards the gravity vector is always safe
        quat_t rotate_down;
        if( imu.get_rotate_down( rotate_down ) == false )
        {
            std::cerr << "Could not get gravity vector, probably timeout" << "\n";
        }


        // Looping over laser by laser (0-15)
        for (const velodyne::Laser &laser : lasers)
        {
            // Distance, azimuth and vertical of laser
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = (laser.azimuth * PI) / 180.0;
            const double vertical = (laser.vertical * PI) / 180.0;

            // Increments frame count and resets pcd count if azimuth angle exceeds field of view end degrees
            if (laser.azimuth > params.fov_end and azimuth_rotation < params.fov_end){
                frame_count++;
                number = 0;
            }

            // azimuth_rotation hold last laser azimuth angle
            azimuth_rotation = laser.azimuth;

            // Check if the points are inside scope (case when fov start is greater than fov end)
            if (laser.azimuth<params.fov_start and laser.azimuth>params.fov_end and params.fov_start>params.fov_end){
                // Don't write pcds and continue
                write_pcd = false;
                continue;
            }

            // Check if the points are inside scope (case when fov end is greater than fov start)
            if ((laser.azimuth<params.fov_start || laser.azimuth>params.fov_end) and params.fov_end>params.fov_start){
                // Don't write pcds and continue
                write_pcd = false;
                continue;
            }

            double x = distance * std::cos(vertical) * std::sin(azimuth); // x-coordinate
            double y = distance * std::cos(vertical) * std::cos(azimuth); // y-coordinate
            double z = distance * std::sin(vertical);                     // z-coordinate
            if( params.apply_correction )
            {
                z = z + vertical_correction[laser.id] / 10.0;
            }

            // Laser intensity
            const auto intensity = static_cast<unsigned int> (laser.intensity);
            // Laser timestamp
            timestamp = laser.time;

            // Points for PCD file
            vec3_t xyz( x / 100.0, y / 100.0, z / 100.0 );

            // Color mapping: intensity -> rgb
            double v = -1.0 + double(intensity) / 75.0;

            vec3_t rgb( 255.0 * clamp(1.5 - std::abs(2.0 * v - 1.0)),
                        255.0 * clamp(1.5 - std::abs(2.0 * v)),
                        255.0 * clamp(1.5 - std::abs(2.0 * v + 1.0)) );

            cloud.points[j].x = xyz(0);
            cloud.points[j].y = xyz(1);
            cloud.points[j].z = xyz(2);
            cloud.points[j].r = rgb(0);
            cloud.points[j].g = rgb(1);
            cloud.points[j].b = rgb(2);

            // adding the laser vertical component to the point label
            cloud.points[j].label = laser.vertical;

            // Increment counter
            j++;
        }

        // If all pcd points are inside scope, then write the pcd
        if (write_pcd and cloud.size() > 0)
        {
            // Setting the filename string
            std::ostringstream filename;
            filename << "scan_" << std::to_string(frame_count) << "_" << std::to_string(number++) + ".pcd";

            // Writing cloud to binary pcd
            boost::filesystem::create_directories(path + "/datapackets");
            pcl::io::savePCDFileBinary(path + "/datapackets/" + filename.str(), cloud);

            // Vector with path and filename of all pcds
            pcds.push_back(path + filename.str());

            // Get quaternion from imu
            quat_t currentQuaternion;
            if( imu.get_quaternion( currentQuaternion ) == false )
            {
                std::cerr << "Could not get quaternion, probably timeout" << endl;
            }

            currentQuaternion.normalize();
            if (!startQuaternionSet) {
                startQuaternion = currentQuaternion;
                startQuaternionSet = true;
            }
            currentQuaternion = (startQuaternion.conjugate() * currentQuaternion);

            // Get gravity vector from imu
            vec3_t gravity;
            if( imu.get_gravity_vector( gravity ) == false )
            {
                std::cerr << "Could not get gravity vector, probably timeout" << "\n";
            }

// currentQuaternion = quat_t( 1.0f, 0.0f, 0.0f, 0.0f );

            // Write quaternion and gravity data to csv file
            quaternion.open(path + "/quaternions/quaternions_datapacket.csv", std::ios_base::app);
            quaternion << currentQuaternion.w() << ","
                       << currentQuaternion.x() << ","
                       << currentQuaternion.y() << ","
                       << currentQuaternion.z() << ", "
                       << gravity(0) << ","
                       << gravity(1) << ","
                       << gravity(2) << ", "
                       << rotate_down.w() << ","
                       << rotate_down.x() << ","
                       << rotate_down.y() << ","
                       << rotate_down.z() << ", "
                       << timestamp << ", "
                       << frame_count
                       << endl;
            quaternion.close();

            vec3_t angv;
            if( imu.get_angular_velocity( angv ) == false )
            {
                std::cerr << "Could not get angular velocity, probably timeout" << "\n";
            }

            vec3_t linacc;
            if( imu.get_linear_acceleration( linacc ) == false )
            {
                std::cerr << "Could not get linear acceleration, probably timeout" << "\n";
            }

            // Write angular velocity and linear acceleration to file
            imu_data.open(path + "/imu/imu_data.csv", std::ios_base::app);
            imu_data << angv(0) << ","
                     << angv(1) << ","
                     << angv(2) << ","
                     << linacc(0) << ","
                     << linacc(1) << ","
                     << linacc(2)
                     << endl;
            imu_data.close();
        }
    }

    return 0;
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

