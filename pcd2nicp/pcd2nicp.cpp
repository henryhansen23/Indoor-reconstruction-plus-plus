//
// pcd2nicp
// Convert a .pcd file to .nicp
//
// pcd is Point Cloud Data from PCL (PointCloud Library).
// nicp is the input for the nicp tools https://github.com/yorsh87/nicp
//
// Carsten Griwodz
//

#include <string>
#include <fstream>
#include <iostream>
#include <iomanip> // setprecision

#include <cstdlib>
#include <cstring>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

using namespace std;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << "Usage: " << argv[0] << " input.pcd output.nicp" << endl;
        return -1;
    }

    string infile_name( argv[1] );
    string outfile_name( argv[2] );

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );

    if( pcl::io::loadPCDFile<pcl::PointXYZ>( infile_name.c_str(), *cloud ) == -1)
    {
        cerr << "Failed to read a point cloud from input file " << infile_name << endl;
        return -1;
    }

    // pcl::PointXYZ min_point;
    // pcl::PointXYZ max_point;
    // pcl::getMinMax3D( *cloud, min_point, max_point );

    ofstream ostr( outfile_name );

    if( ! ostr.good() )
    {
        cerr << "Failed to open file named " << outfile_name << " for NICP point cloud output" << endl;
        return -1;
    }

    ostr << "NICPCLOUD " << cloud->size() << " 0" << endl;

    ostr << "0 0 0 0 0 0" << endl;

    for( auto it = cloud->begin(); it != cloud->end(); it++ )
    {
        const pcl::PointXYZ& point = *it;
        ostr << "POINTWITHSTAT "
             << point.x << " " << point.y << " " << point.z
             << " 1 0 0"
             << " 1 0 0 0"
             << " 0 1 0 0"
             << " 0 0 1 0"
             << " 0 0 0 1" << endl;
    }

    return 0;
}

