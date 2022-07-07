#include <iostream>

#include <pcl/pcl_config.h>

#include <boost/program_options.hpp>

#include "cmdline.h"

namespace po = boost::program_options;

CmdLine::CmdLine( int argc, char** argv )
    : _visualize( false )
    , _icp( "generalized" )
{
    po::positional_options_description p;
    p.add("data-dir", -1);

    po::options_description desc{"Options"};
    desc.add_options()
        ( "help,h",
          "Help screen" )
        ( "verbose,v",
          po::bool_switch(&_visualize)->default_value(false),
          "Visualize the reconstruction" )
        ( "icp",
          po::value<std::string>(&_icp),
          "The ICP algorithm to use: generalized, non-linear or symmetric (only with PCL>=1.10)" )
        ( "data-dir,d",
          po::value<std::string>(&_data_dir)->required(),
          "Directory where pointcloud fragments can be found (positional argument)" )
        ;

    po::variables_map vm;

    try
    {
        po::store(
            po::command_line_parser(argc, argv).options(desc).positional(p).run(),
            vm );

        po::notify(vm);

        if (vm.count("help"))
        {
            std::cerr << desc << std::endl;
            exit( 0 );
        }
        else if (vm.count("icp"))
        {
            if( ( vm["icp"].as<std::string>() != "generalized" ) &&
                ( vm["icp"].as<std::string>() != "non-linear" )
                #if PCL_VERSION_COMPARE(>, 1, 10, 0)
                && ( vm["icp"].as<std::string>() != "symmetric" )
                #endif
                )
            {
                std::cerr << "Incorrect parameter for ICP option." << std::endl
                        << desc << std::endl;
            }
        }
    }
    catch (const po::error &ex)
    {
        std::cerr << std::endl
                << "Error: " << ex.what() << std::endl
                << std::endl
                << desc << std::endl;
        exit( -1 );
    }
}

