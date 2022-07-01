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

    std::string directory;
    std::string path;
    std::ofstream quaternion;
    std::ofstream imu_data;
    std::string fragment;
    std::string odometry;
    int laser_num;
    int fov_start;
    int fov_end;
    bool write_pcd;
    bool apply_correction;
    std::vector <std::string> pcds;

    // Command line arguments
    char opt_d[] = "-d";
    char opt_f[] = "-f";
    char opt_o[] = "-o";
    char opt_s[] = "-start";
    char opt_e[] = "-end";
    char opt_h[] = "-h";
    char opt_c[] = "-c";

    pcl::console::parse_argument(argc, (char**)argv, opt_d, directory);
    pcl::console::parse_argument(argc, (char**)argv, opt_f, fragment);
    pcl::console::parse_argument(argc, (char**)argv, opt_o, odometry);
    pcl::console::parse_argument(argc, (char**)argv, opt_s, fov_start);
    pcl::console::parse_argument(argc, (char**)argv, opt_e, fov_end);
    pcl::console::parse_argument(argc, (char**)argv, opt_c, apply_correction);

    if(argc==1) {
        usage(argv[0]);
        return 0;
    }

    // Check correct command line arguments
    if (pcl::console::find_switch(argc, (char**)argv, opt_h)) {
        usage(argv[0]);
        return 0;
    }

    if (directory.empty()) {
        cout << "\033[1;31mSpecify directory name\033[0m\n";
        return 0;
    }

    if (fragment.empty() and odometry.empty()) {
        cout << "\033[1;31mSpecify fragment or movement number\033[0m\n";
        return 0;
    }

    if (fov_start < 0 or fov_start > 359) {
        cout << "\033[1;31mSpecify correct field of view start degrees (range: 0 -> 359)\033[0m\n";
        return 0;
    }

    if (fov_end < 0 or fov_end > 359) {
        cout << "\033[1;31mSpecify correct field of view end degrees (range: 0 -> 359)\033[0m\n";
        return 0;
    }

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

    // Setting correct fragment path
    if (pcl::console::find_switch(argc, (char**)argv, "-f")) {
        path = "../../pcds/" + directory + "/fragments/fragment_" + fragment + "/";
        std::cout << "Writing to: fragment_" + fragment << "\n\n";
    }

    // Setting correct odometry path
    else if (pcl::console::find_switch(argc, (char**)argv, "-o")) {
        path = "../../pcds/" + directory + "/odometry/odometry_" + odometry + "/";
        std::cout << "Writing to: odometry_" + odometry << "\n\n";
    }

    // Create directories in path
    cout << "Creating the path " << path << endl;
    boost::filesystem::create_directories(path);

    // Open and write header to quaternion csv file
    boost::filesystem::create_directories(path + "/quaternions");
    quaternion.open(path + "/quaternions/quaternions_datapacket.csv");
    quaternion << "q_w,q_x,q_y,q_z,t,c\n";
    quaternion.close();

    // Open and write header to imu data csv file
    boost::filesystem::create_directories(path + "/imu");
    imu_data.open(path + "/imu/imu_data.csv");
    imu_data << "angv_x,angv_y,angv_z,lina_x,lina_y,lina_z\n";
    imu_data.close();

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

    int ct = 1000;
    while (capture.isRun() && !interrupted) {
        ct--;
        if( ct<0 ) { cerr << "."; ct = 1000; }

        // Initialize lasers with velodyne data
        std::vector <velodyne::Laser> lasers;
        capture >> lasers;
        if (lasers.empty()) {
            continue;
        }
        // Fill in the cloud data -> used for writing to pcd file
        cloud.width = static_cast <uint32_t>(lasers.size());
        cloud.height = 1;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);

        // Bool used to write pcds if pcd is inside given fov scope
        write_pcd = true;

        // Initialize counter
        int j = 0;

        // Looping over laser by laser (0-15)
        for (const velodyne::Laser &laser : lasers) {
            // Distance, azimuth and vertical of laser
            const auto distance = static_cast<double>( laser.distance );
            const double azimuth = (laser.azimuth * PI) / 180.0;
            const double vertical = (laser.vertical * PI) / 180.0;

            // Increments frame count and resets pcd count if azimuth angle exceeds field of view end degrees
            if (laser.azimuth > fov_end and azimuth_rotation < fov_end){
                frame_count++;
                number = 0;
            }

            // azimuth_rotation hold last laser azimuth angle
            azimuth_rotation = laser.azimuth;

            // Check if the points are inside scope (case when fov start is greater than fov end)
            if (laser.azimuth<fov_start and laser.azimuth>fov_end and fov_start>fov_end){
                // Don't write pcds and continue
                write_pcd = false;
                continue;
            }

            // Check if the points are inside scope (case when fov end is greater than fov start)
            if ((laser.azimuth<fov_start || laser.azimuth>fov_end) and fov_end>fov_start){
                // Don't write pcds and continue
                write_pcd = false;
                continue;
            }

            // x-coordinate
            auto x = static_cast <float> ((distance * std::cos(vertical)) * std::sin(azimuth));
            // y-coordinate
            auto y = static_cast <float> ((distance * std::cos(vertical)) * std::cos(azimuth));
            // z-coordinate
            auto z = static_cast <float> ((distance * std::sin(vertical)));
            // Laser intensity
            const auto intensity = static_cast <unsigned int> (laser.intensity);
            // Laser timestamp
            timestamp = laser.time;

            // Points for PCD file
            cloud.points[j].x = x / 100; // converting to meters
            cloud.points[j].y = y / 100; // converting to meters
            if (apply_correction)
            {
                cloud.points[j].z = (z + (vertical_correction[laser.id] / 10)) / 100; // converting to meters
            } else
            {
                cloud.points[j].z = z / 100; // converting to meters
            }

            // Color mapping: intensity -> rgb
            double v = -1 + double(intensity) / 75;
            double r = clamp(1.5 - std::abs(2.0 * v - 1.0));
            double g = clamp(1.5 - std::abs(2.0 * v));
            double b = clamp(1.5 - std::abs(2.0 * v + 1.0));

            // making hex string of rgb values
            std::stringstream stream;
            stream << "0x00";
            stream << std::setfill ('0') << std::setw(2) << std::hex << int(r*255);
            stream << std::setfill ('0') << std::setw(2) << std::hex << int(g*255);
            stream << std::setfill ('0') << std::setw(2) << std::hex << int(b*255);
            std::string rgb_hex(stream.str());

            // convert hex string to float
            uint32_t num;
            float rgb;
            char cstr[rgb_hex.size() + 1];
            std::strcpy(cstr, rgb_hex.c_str());
            std::sscanf(cstr, "%x", &num);
            rgb = *((float*)&num);

            // adding rgb float to point cloud
            cloud.points[j].rgb = rgb;

            // adding the laser vertical component to the point label
            cloud.points[j].label = laser.vertical;

            // Increment counter
            j++;
        }

        // If all pcd points are inside scope, then write the pcd
        if (write_pcd and cloud.size() > 0) {
            // Setting the filename string
            std::ostringstream filename;
            filename << "scan_" << std::to_string(frame_count) << "_" << std::to_string(number++) + ".pcd";

            // Writing cloud to binary pcd
            boost::filesystem::create_directories(path + "/datapackets");
            pcl::io::savePCDFileBinary(path + "/datapackets/" + filename.str(), cloud);

            // Vector with path and filename of all pcds
            pcds.push_back(path + filename.str());

            // Get quaternions from imu
            int16_t q_w, q_x, q_y, q_z;

            if (imu_v2_get_quaternion(&imu, &q_w, &q_x, &q_y, &q_z) < 0) {
              std::cerr << "Could not get quaternion, probably timeout" << "\n";
            }

            // Write quaternion data to csv file
            quaternion.open(path + "/quaternions/quaternions_datapacket.csv", std::ios_base::app);
            quaternion << q_w / 16383.0 << "," << q_x / 16383.0 << "," << q_y / 16383.0 << "," << q_z / 16383.0
                      << "," << timestamp << "," << frame_count <<"\n";
            quaternion.close();

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

            // Write angular velocity and linear acceleration to file
            imu_data.open(path + "/imu/imu_data.csv", std::ios_base::app);
            imu_data << angv_x / 16.0  << "," << angv_y / 16.0  << "," << angv_z / 16.0  << ","
                    << lina_x / 100.0 << "," << lina_y / 100.0 << "," << lina_z / 100.0 <<"\n";
            imu_data.close();
        }
    }

    imu_v2_destroy(&imu);
    ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally
    return 0;
}

static void usage( const char* prog_name )
{
     std::cout << "\n\nUsage: "<<prog_name<<" [options]\n\n"
                  "This program uses only the VLP-16 Lidar and the Tinkerforge IMU 2.0 to record\n"
                  "pointcloud data. The Lidar data and IMU data are not synchronized, that would\n"
                  "require the addition of a common time source as documented in the VLP-16 docs.\n\n"
                  "Options:\n"
                  "-------------------------------------------------------------------------------\n"
                  "-h             This help message\n"
                  "-d <dir_name>  Directory name of where to save the pcds\n"
                  "-f <int>       Specify fragment number\n"
                  "-o <int>       or: specify odometry number\n"
                  "-start <int>   Specify field of view start degree [0-359]\n"
                  "-end <int>     Specify field of view end degree [0-359]\n"
                  "-c             Apply vertical correction\n"
                  "-------------------------------------------------------------------------------\n"
                  "\n"
                  "-o : Use this if you move with the VLP-16 while recording\n"
                  "-f : Use this if you stand still and use the tripod sweep\n"
                  "     -o and -f are mutually exclusive.\n"
                  "-start and -end restrict the recording angle. That allows you to prevent\n"
                  "                you from recording yourself.\n"
                  "-d :  creates the base directory where the recorded PCD files are stored.\n"
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

