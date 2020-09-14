# Indoor-reconstruction-plus-plus
Indoor reconstruction with Velodyne Lidar and IMU.

## Notes:
Requires:
* C++ compiler
* boost
* PCL with dependencies 
* CMake

## Setup 
Install a C++ compiler, CMake and PCL with dependencies. 
Compile by running $ sh compile.sh

## Information:
### *interpolation_data_collection*:<a name="interpolate"></a>
#### *build*:<a name="interpolate"></a>

This script is built upon [UnaNancyOwen's simple program](https://github.com/UnaNancyOwen/VelodyneCapture/tree/master/sample/simple) and uses his VelodyneCapture class. It retrieves data from the VLP-16 in forms of *datapackets*, which is approximately *2.38&deg;* of a full *360&deg;* scan at *300 RPM*. One datapacket contains data from *24* firing sequences of the 16 lasers, which results in a maximum of *384* points per packet. 

All data packets are written to a binary pcd file the moment it is retrieved, and an IMU measurement is addressed to a separate line in a CSV file. The point cloud is colored based on the intensity return, where the intensity value is converted to RGB float with a color mapping procedure. 

A fragment is a stationary sweep while an odometry is a measurement done while moving to estimate the displacement.
Odometry measurements can be done with normal walking speed. 

The captured data will be stored in the build folder. 

How to use this script with command line arguments:

| Syntax      | Description |
| ----------- | ----------- |
| -h      | Prints a help message|
| -d   | Directory name of where to save the pcds |
| -f   | Specify fragment number (0,1,2,...,n)|
| -o   | Specify odometry number (01,12,23,...,n)|
| -start   | Specify field of view start degree [0-359]|
| -end   | Specify field of view end degree [0-359] |

Example usage: 

`./interpolation_vlp -d data_dir -f 0 -start 270 -end 90`

press **ctrl c** to stop the data collection. 

### *build*:<a name="interpolate"></a>

This program processes all collected point cloud data and IMU data in a data folder.
Datapackets are turned into fragments and each fragment is then visualized.
The odometry translation between the fragments is estimated with non-linear ICP. 
A rough computation time estimate for odometry is 1 minute for every 5 meters. 
The estimated translations are used for initial alignment for the Generalized ICP whichs align the fragments. 

To reconstruct the captured scene, run the "reconstruct"-execuatable by:

`./reconstruct data_dir` 

or with "v" for visualization of the fragment registration: 

`./reconstruct data_dir v` 

