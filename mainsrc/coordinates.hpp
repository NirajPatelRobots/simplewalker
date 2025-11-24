/* useful stuff for simplewalker
July 2022
TODO:
*/

#ifndef WALKER_COORDINATES_HPP
#define WALKER_COORDINATES_HPP

#include <cmath>
#include <string>
#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include "walkertypes.h"
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
const Eigen::Array3f RIGHTSCALE{1., -1., 1.}; // multiply arrays by RIGHTSCALE to change right with left
const Vector3f GRAVITY_ACCEL{-9.81 * UP_DIR};


inline Vector3f project_zeroZ(const Vector3f &input) {
    return {input(0),input(1),0};
}

#endif  // WALKER_COORDINATES_HPP
