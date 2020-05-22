

#include <vector> 


#include <Eigen/dense>


#include "data_types.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


std::vector <Quaternion_file> interpolate_quaternions(const std::vector <Quaternion_file> & quaternions) {


                              Eigen::Vector4d previous_quaternion {1000, 1000, 1000, 1000};  // initialization of variable


                              double diff_time, previous_time = 0;

                              Eigen::Vector4d a, b, q, diff_quat; 


                              std::vector <Quaternion_file> interpolated_quaternions; 


                              std::vector <Eigen::Vector4d> equal_quaternions; 

                              std::vector <double> timestamp; 


                              for (std::size_t i = 0; i < quaternions.size(); ++i) {


                                  if (previous_quaternion.isApprox(quaternions[i].quat)) {
                                    

                                     if (equal_quaternions.empty()) {
                                         

                                        equal_quaternions.push_back(previous_quaternion);

                                        interpolated_quaternions.pop_back();

                                        timestamp.push_back(previous_time);


                                     }


                                     equal_quaternions.push_back(quaternions[i].quat);

                                     timestamp.push_back(quaternions[i].time);


                                  }


                                  else {


                                       if (!equal_quaternions.empty()) {


                                          diff_quat = quaternions[i].quat - previous_quaternion;
  
                                          diff_time = quaternions[i].time - timestamp[0];

                                        
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

                                              interpolated_quaternions.push_back({q, timestamp[j]});


                                          }


                                          interpolated_quaternions.push_back({quaternions[i].quat, quaternions[i].time});


                                          equal_quaternions.clear(); 

                                          timestamp.clear(); 

                                          previous_quaternion = quaternions[i].quat; 

                                          previous_time = quaternions[i].time;

                          
                                       }


                                       else {


                                            interpolated_quaternions.push_back({quaternions[i].quat, quaternions[i].time});

                                            previous_quaternion = quaternions[i].quat;

                                            previous_time = quaternions[i].time;


                                       }

  
                                  }


                              }

           
                              // Appending the last quaternions if they are all equal

                              if (!equal_quaternions.empty()) {


                                 for (std::size_t i = 0; i < equal_quaternions.size(); ++i) {
  
                                     interpolated_quaternions.push_back({equal_quaternions[i], timestamp[i]});

                                 }


                              }
                                            
                              
                              return interpolated_quaternions; 


}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

