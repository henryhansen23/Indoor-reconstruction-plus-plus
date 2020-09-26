#include <utility>
#include <vector>
#include <iostream>

#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

#include "quaternion_interpolation.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
interpolate_quaternions( vector4d_t& interpolated_quaternions,
                         const quart_vector_t& quaternions_time )
{
    quart_vector_t interpolated_quaternions_time;
    vector4d_t equal_quaternions;
std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
interpolate_quaternions(const std::vector<std::pair<Eigen::Vector4d, double>, Eigen::aligned_allocator<std::pair<Eigen::Vector4d, double>> > &quaternions_time) {
    std::vector<std::pair<Eigen::Vector4d, double>, Eigen::aligned_allocator<std::pair<Eigen::Vector4d, double>>> interpolated_quaternions_time;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> equal_quaternions;
    std::vector<double> timestamp;
    Eigen::Vector4d previous_quaternion{1000, 1000, 1000, 1000};  // initialization of variable
    Eigen::Vector4d a, b, q, diff_quat;

    double diff_time, previous_time = 0;
    for (std::size_t i = 0; i < quaternions_time.first.size(); ++i) {
        if (previous_quaternion.isApprox(quaternions_time.first[i])) {
            if (equal_quaternions.empty()) {
                equal_quaternions.push_back(previous_quaternion);
                interpolated_quaternions_time.first .pop_back();
                interpolated_quaternions_time.second.pop_back();
                timestamp.push_back(previous_time);
            }

            equal_quaternions.push_back(quaternions_time.first[i]);
            timestamp        .push_back(quaternions_time.second[i]);
        } else {
            if (!equal_quaternions.empty()) {
                diff_quat = quaternions_time.first [i] - previous_quaternion;
                diff_time = quaternions_time.second[i] - timestamp[0];

                // Slope
                a << diff_quat(0) / diff_time,
                        diff_quat(1) / diff_time,
                        diff_quat(2) / diff_time,
                        diff_quat(3) / diff_time;

                // y-intercept
                b << equal_quaternions[0](0) - a(0) * timestamp[0],
                        equal_quaternions[0](1) - a(1) * timestamp[0],
                        equal_quaternions[0](2) - a(2) * timestamp[0],
                        equal_quaternions[0](3) - a(3) * timestamp[0];

                for (int j = 0; j < timestamp.size(); ++j) {
                    // y = ax + b
                    q << a(0) * timestamp[j] + b(0),
                            a(1) * timestamp[j] + b(1),
                            a(2) * timestamp[j] + b(2),
                            a(3) * timestamp[j] + b(3);
                    interpolated_quaternions_time.first .push_back( q );
                    interpolated_quaternions_time.second.push_back( timestamp[j] );
                }

                interpolated_quaternions_time.first .push_back( quaternions_time.first [i] );
                interpolated_quaternions_time.second.push_back( quaternions_time.second[i] );

                equal_quaternions.clear();
                timestamp.clear();
                previous_quaternion = quaternions_time.first [i];
                previous_time       = quaternions_time.second[i];
            } else {
                interpolated_quaternions_time.first .push_back( quaternions_time.first [i] );
                interpolated_quaternions_time.second.push_back( quaternions_time.second[i] );
                previous_quaternion = quaternions_time.first [i];
                previous_time       = quaternions_time.second[i];
            }
        }
    }

    // Appending the last quaternions if they are all equal
    if (!equal_quaternions.empty()) {


        for (std::size_t i = 0; i < equal_quaternions.size(); ++i) {

            interpolated_quaternions_time.first .push_back( equal_quaternions[i] );
            interpolated_quaternions_time.second.push_back( timestamp[i] );

        }


    }


    // Remove time


    interpolated_quaternions.resize( interpolated_quaternions_time.first.size() );

    for (std::size_t i = 0; i < interpolated_quaternions_time.first.size(); ++i) {

        interpolated_quaternions[i] = interpolated_quaternions_time.first[i];


    }


}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

