

#include <vector> 


#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


#include "data_types.h"

#include "quaternion_operations.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void make_rotation_matrix(const Eigen::Vector4d quaternion, Eigen::Matrix3d & rotation) {


     Eigen::Vector4d quaternion_normalized; 

     normalize_quaternion(quaternion, quaternion_normalized); 

     make_rotation_matrix_from_quaternion(quaternion_normalized, rotation); 


}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void make_translation_vector(const Eigen::Matrix3d rotation, Eigen::Vector3d & tripod_position) {

    
     Eigen::Vector3d tripod {0, 0, 0.101}; // LiDAR is located 0.101m from tripod joint

     tripod_position = rotation * tripod;
 

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


std::vector <Eigen::Matrix4d> make_transformation_matrices(const std::vector <std::pair <Eigen::Vector4d, double> > & quaternions) {


                              std::vector <Eigen::Matrix4d> matrices;


                              Eigen::Matrix3d tripod_rotation; 

                              Eigen::Vector3d tripod_position;



                              for (std::size_t i = 0; i < quaternions.size(); ++i) {


                                  make_rotation_matrix(quaternions[i].first, tripod_rotation); 

                                  make_translation_vector(tripod_rotation, tripod_position);  


                                  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity(); 

                                  matrix.topLeftCorner <3,3> () = tripod_rotation;
  
                                  matrix.col(3).head <3> () = tripod_position;


                                  matrices.push_back(matrix); 

 
                              }  


                              return matrices; 


}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

