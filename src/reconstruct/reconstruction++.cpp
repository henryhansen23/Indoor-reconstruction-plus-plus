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
typedef pcl::PointCloud<pcl::PointXYZL> point_cloud_w_labels;

///////////////////////////////////////////////////////////////////////////////////////////////////

int
main(int argc, char **argv)
{
    //////////////////////////////// Command line arguments /////////////////////////////////////////
    CmdLine cmdline(argc, argv);
    const std::string &data_dir = cmdline.getDataDir(); // argv[1];
    bool visualization = cmdline.getVisualize(); // false;

    //////////////////////////////////// Fragments ///////////////////////////////////////////////////
    std::cout << std::endl << "Fragments" << std::endl << std::endl;
    int fragments = number_of_directories(data_dir + "/fragments");

    vec3_vector_t translations;

    for( boost::filesystem::directory_iterator itr( data_dir + "/fragments" ); itr!=boost::filesystem::directory_iterator(); ++itr )
    {
        std::cout << itr->path().filename() << ' '; // display filename only
        if (boost::filesystem::is_regular_file(itr->status())) std::cout << " [" << boost::filesystem::file_size(itr->path()) << ']';
        std::cout << std::endl;

        std::string fragment = itr->path().string();

        // Read quaternions
        std::cout << "[frag] Reading quaternions file ... "<< std::flush;
        quart_vector_t quaternions_time;
        read_quaternions_file( quaternions_time,
                               fragment + "/quaternions" );
        std::cout << "done." << std::endl;

        // Interpolate quaternions
        std::cout << "[frag] Interpolating quaternions ... "<< std::flush;
        vec4d_vector_t interpolated_quaternions;
        interpolate_quaternions( interpolated_quaternions, quaternions_time );
        std::cout << "done." << std::endl;

        // Load datapackets
        std::cout << "[frag] Loading datapackets ... "<< std::flush;
        std::vector<std::vector<point_cloud_w_labels> >
            datapackets_clouds = load_datapackets(fragment + "/datapackets");
        std::cout << "done." << std::endl;

        // Combine datapackets to fragment
        std::cout << "[frag] Combining datapackets to fragment ... "<< std::flush;
        combine_datapackets_to_fragment( datapackets_clouds,
                                         interpolated_quaternions,
                                         fragment + "/fragment.pcd",
                                         cmdline.getVisualize() );
        std::cout << "done." << std::endl;


        vec3_t translation{0, 0, 0};
        translations.push_back(translation);
    }

    std::cout << std::endl << std::endl;

    //////////////////////////////////// Odometry ///////////////////////////////////////////////////////////
    if (boost::filesystem::exists(data_dir + "/odometry")) {
        std::cout << "Odometry" << std::endl << std::endl;
        int odometries = number_of_directories(data_dir + "/odometry");
        for (int i = 0; i < odometries; ++i) {
            std::cout << i << std::endl;
            std::string odometry = "odometry_" + std::to_string(i);

            // Read quaternions
            std::cout << "Reading quaternions file..."<< std::flush;
            quart_vector_t quaternions_time;
            read_quaternions_file( quaternions_time,
                                   data_dir + "/odometry/" + odometry + "/quaternions" );
            std::cout << "Done." << std::endl;

            // Interpolate quaternions
            std::cout << "Interpolating quaternions..."<< std::flush;
            vec4d_vector_t interpolated_quaternions;
            interpolate_quaternions( interpolated_quaternions, quaternions_time );
            std::cout << "Done." << std::endl;

            // Load datapackets
            std::cout << "Loading datapackets..."<< std::endl;
            std::vector<std::vector<point_cloud_w_labels> >
                datapacket_clouds = load_datapackets(data_dir + "/odometry/" + odometry + "/datapackets");
            std::cout << "Done." << std::endl;

            // Combine datapackets to scans
            std::cout << "Combining datapackets to scans..."<< std::flush;
            combine_datapackets_to_scans(datapacket_clouds,
                                         interpolated_quaternions,
                                         data_dir + "/odometry/" + odometry);
            std::cout << "Done." << std::endl;

            // Estimate translation
            std::cout << "Estimating translation..."<< std::flush;
            vec3_t translation{0, 0, 0};
            translation_estimation(data_dir + "/odometry/" + odometry + "/scans", translation);
            translations.push_back(translation);
            std::cout << "Done." << std::endl;
        }
        std::cout << std::endl << std::endl;

        /////////////////////////// Fragment pairwise registration with odometry ////////////////////////////////////////////////////////
        std::cout << "Fragment pairwise registration" << std::endl << std::endl;
        std::vector<point_cloud> fragment_clouds = load_fragments( data_dir );
        incremental_pairwise_registration(fragment_clouds,
                                          translations,
                                          data_dir + "/combined_cloud.pcd",
                                          cmdline.getICPType(),
                                          visualization);
    }
#if 0
    /*
     * Merging point clouds from several independent scans does not work.
     * It crashes eventually. Fix before re-enabling.
     */
    else
    {
        std::cout << "Fragment pairwise registration" << std::endl << std::endl;
        std::vector<point_cloud> fragment_clouds = load_fragments( data_dir );
        incremental_pairwise_registration(fragment_clouds,
                                          translations,
                                          data_dir + "/combined_cloud.pcd",
                                          cmdline.getICPType(),
                                          visualization);
    }
#endif
    //////////////////////////////////////////// END ////////////////////////////////////////////////////////////////////////
    return 0;
}

