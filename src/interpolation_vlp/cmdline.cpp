#include <iostream>

// Boost
#include <boost/program_options.hpp>

#include "cmdline.hpp"

void parseargs( int argc, char** argv, Parameters& params )
{
    using namespace boost::program_options;

    options_description options("Options");
    {
        options.add_options()
            ( "help,h", "Print usage" )
            ( "directory,d",
              value<std::string>( &params.directory )->required(),
              "Directory name of where to save the pcds" )
            ( "odometry,o",
              value<int>()->notifier( [&](int v) { params.setOdometry(v); } ),
              "Specify odometry number of this scan" )
            ( "start,s",
              value<float>(&params.fov_start)->default_value( 0.0f ),
              "Specify field of view start degree [0-359]" )
            ( "end,e",
              value<float>(&params.fov_end)->default_value( 359.0f ),
              "Specify field of view end degree [0-359]" )
            ( "wait-gps",
              bool_switch(&params.wait_gps)->default_value( false ),
              "Specify if the scanner should wait for GPS fix" )
            ( "correction,c",
              bool_switch(&params.apply_correction)->default_value( false ),
              "Apply vertical correction" )
            ;
    }

    options_description all("Allowed options");
    all.add(options);
    variables_map vm;

    try
    {
       store(parse_command_line(argc, argv, all), vm);

       if (vm.count("help")) {
            std::cout << "\n\nUsage: "<< argv[0] <<" [options]\n\n"
                      << "This program uses only the VLP-16 Lidar and the Tinkerforge IMU 2.0 to record\n"
                      << "pointcloud data. The Lidar data and IMU data are not synchronized, that would\n"
                      << "require the addition of a common time source as documented in the VLP-16 docs.\n"
                      << "The Tinkerforge GPS 2.0 is installed but does not work yet.\n\n"
                      << all
                      << std::endl
                      << "-o : Use this if you move with the VLP-16 while recording\n"
                      << "     otherwise a vertical sweep on a tripod is assumed.\n"
                      << "--start and --end restrict the recording angle. That allows you to prevent\n"
                      << "                you from recording yourself.\n"
                      << "-d :  creates the base directory where the recorded PCD files are stored.\n"
                      << std::endl
                      << std::endl;
           exit(EXIT_SUCCESS);
       }

        notify(vm); // Notify does processing (e.g., raise exceptions if required args are missing)
    }
    catch(boost::program_options::error& e)
    {
        std::cerr << "Error: " << e.what() << std::endl << std::endl;
        std::cerr << "Usage:\n\n" << all << std::endl;
        exit(EXIT_FAILURE);
    }

}

