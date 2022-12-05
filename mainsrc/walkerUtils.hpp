/* useful stuff for simplewalker
July 2022
TODO:
    vector_statistic
    scalar_statistic unit
    BUG: Jac_rotated_wrt_axis_angle
    scalar_statistic std_dev could be nan
*/

#ifndef WALKER_UTILS_H
#define WALKER_UTILS_H

#include <cmath>
#include <string>
#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "walkertypes.h"
using Eigen::Matrix3f, Eigen::Vector3f, std::cout, std::endl, std::string;
using std::unique_ptr, std::shared_ptr, std::make_unique, std::make_shared;

#ifndef M_PI  // I can't believe I have to do this
const float M_PI {3.14159265358979323846};
const float M_PI_2 {1.57079632679489661923};
#endif
// array indexes
const int LEFT_IDX = 1, FORWARD_IDX = 0, UP_IDX = 2;
const Vector3f RIGHT_DIR{0,-1,0};
const Vector3f LEFT_DIR{0,1,0};
const Vector3f FORWARD_DIR{1,0,0};
const Vector3f UP_DIR{0,0,1};
const Vector3f HORIZONTAL_DIR = LEFT_DIR + FORWARD_DIR;
const Vector3f RIGHTSCALE{-1., 1., 1.}; // multiply vectors by RIGHTSCALE to change right with left


struct scalar_statistic {
    float mean{0};
    float max {0};
    float std_dev{0};
    float most_recent{0};
    unsigned num_data{0};
    unsigned num_bad{0};
    std::string unit{};
    bool update(float new_val);  // returns whether statistics were updated, false if invalid input
    bool is_max(void) const {return (most_recent == max);}
};

std::ostream& operator<<(std::ostream& os, const scalar_statistic& data);

struct Vector_statistic{ //TODO
    Vector3f mean{0, 0, 0};
    Vector3f max{0, 0, 0};
    Matrix3f covariance{Matrix3f::Identity()};
    Vector3f most_recent{0, 0, 0};
    unsigned num_data{0};
    unsigned num_bad{0};
    bool update(Vector3f new_val);
    bool is_max(void) const {return (most_recent == max);}
};

// math
/* find the Jacobian of a rotated vector with respect to the axis-angle vector.
axis_angle is the vector whose direction is the axis of rotation.
    the robot_state axis_angle is from body to world frame. That corresponds to unrotated_vector in body coordinates.
    if you want the jacobian of a vector in body frame, then axis_angle should be the - of the body to world axis_angle in robot_state.
    the magnitude of axis_angle is the angle of rotation.
unrotated_vector is the vector before rotation. 
sets 3x3 Jacobian  */
void Jac_rotated_wrt_axis_angle(Eigen::Ref<Matrix3f> Jacobian, const Vector3f &axis_angle, const Vector3f &unrotated_vector);
/* Given a vector, calculate its unit vector and the jacobian of the unit vector wrt the vector
*/
void jacobian_unitvec_wrt_vec(Matrix3f &jacobian, const Vector3f &vec);

#endif
