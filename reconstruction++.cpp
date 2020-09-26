#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <pcl/point_types.h>


#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

#include <boost/filesystem.hpp>

// #include "cmd_line_parser.h"
#include "load_data.h"
#include "quaternion_interpolation.h"
#include "combine_datapackets.h"
#include "registration_estimation.h"
#include "cmdline.h"

typedef pcl::PointCloud<pcl::PointXYZ> point_cloud;

///////////////////////////////////////////////////////////////////////////////////////////////////

int
main(int argc, char **argv)
{
    //////////////////////////////// Command line arguments /////////////////////////////////////////
    CmdLine cmdline(argc, argv);
    const std::string &data_dir = cmdline.getDataDir(); // argv[1];
    bool visualization = cmdline.getVisualize(); // false;
    // if (argc > 2) {std::string arg = argv[2]; if (arg == "v") {visualization = true;}}

    //////////////////////////////////// Fragments ///////////////////////////////////////////////////
    std::cout << std::endl << "Fragments" << std::endl << std::endl;
    int fragments = number_of_directories(data_dir + "/fragments");

    for (int i = 0; i < fragments; ++i) {
        std::cout << i << std::endl;
        std::string fragment = "fragment_" + std::to_string(i);

        // Read quaternions
        std::cout << "Reading quaternions file..."<< std::endl;
        quart_vector_t quaternions_time;
        read_quaternions_file( quaternions_time,
                               data_dir + "/fragments/" + fragment + "/quaternions" );
        std::cout << "Done." << std::endl;

        // Interpolate quaternions
        std::cout << "Interpolating quaternions..."<< std::endl;
        vector4d_t interpolated_quaternions;
        interpolate_quaternions( interpolated_quaternions, quaternions_time );
        std::cout << "Done." << std::endl;

        // Load datapackets
        std::cout << "Loading datapackets..."<< std::endl;
        std::vector<std::vector<point_cloud> >
            datapackets_clouds = load_datapackets(data_dir + "/fragments/" + fragment + "/datapackets");
        std::cout << "Done." << std::endl;

        // Combine datapackets to fragment
        std::cout << "Combining datapackets to fragment..."<< std::endl;
        combine_datapackets_to_fragment(datapackets_clouds,
                                        interpolated_quaternions,
                                        data_dir + "/fragments/" + fragment);
        std::cout << "Done." << std::endl;
    }

    std::cout << std::endl << std::endl;

    //////////////////////////////////// Odometry ///////////////////////////////////////////////////////////
    if (boost::filesystem::exists(data_dir + "/odometry")) {
        std::cout << "Odometry" << std::endl << std::endl;
        int odometries = number_of_directories(data_dir + "/odometry");
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > translations;
        for (int i = 0; i < odometries; ++i) {
            std::cout << i << std::endl;
            std::string odometry = "odometry_" + std::to_string(i);

            // Read quaternions
            std::cout << "Reading quaternions file..."<< std::endl;
            quart_vector_t quaternions_time;
            read_quaternions_file( quaternions_time,
                                   data_dir + "/odometry/" + odometry + "/quaternions" );
            std::cout << "Done." << std::endl;

            // Interpolate quaternions
            vector4d_t interpolated_quaternions;
            interpolate_quaternions( interpolated_quaternions, quaternions_time );
            std::cout << "Done." << std::endl;

            // Load datapackets
            std::cout << "Loading datapackets..."<< std::endl;
            std::vector<std::vector<point_cloud> >
                datapacket_clouds = load_datapackets(data_dir + "/odometry/" + odometry + "/datapackets");
            std::cout << "Done." << std::endl;

            // Combine datapackets to scans
            std::cout << "Combining datapackets to scans..."<< std::endl;
            combine_datapackets_to_scans(datapacket_clouds,
                                         interpolated_quaternions,
                                         data_dir + "/odometry/" + odometry);
            std::cout << "Done." << std::endl;

            // Estimate translation
            std::cout << "Estimating translation..."<< std::endl;
            Eigen::Vector3f translation{0, 0, 0};
            translation_estimation(data_dir + "/odometry/" + odometry + "/scans", translation);
            translations.push_back(translation);
            std::cout << "Done." << std::endl;
        }
        std::cout << std::endl << std::endl;

        /////////////////////////// Fragment pairwise registration with odometry ////////////////////////////////////////////////////////
        std::cout << "Fragment pairwise registration" << std::endl << std::endl;
        std::vector<point_cloud> fragment_clouds = load_fragments(data_dir + "/fragments", fragments);
        incremental_pairwise_registration(fragment_clouds,
                                          translations,
                                          data_dir,
                                          cmdline.getICPType(),
                                          visualization);
    }
    //////////////////////////////////////////// END ////////////////////////////////////////////////////////////////////////
    return 0;
}
