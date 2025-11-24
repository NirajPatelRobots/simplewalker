/* rotation for simplewalker
July 2022
TODO:
    BUG: Jac_rotated_wrt_axis_angle nan at axis = 0 (math singularity problem)
*/

#ifndef WALKER_ROTATION_HPP
#define WALKER_ROTATION_HPP

#include "../mainsrc/coordinates.hpp"


/* find the Jacobian of a rotated vector with respect to the axis-angle vector.
axis_angle is the vector whose direction is the axis of rotation.
    the robot_state axis_angle is from body to world frame. That corresponds to unrotated_vector in body coordinates.
    if you want the jacobian of a vector in body frame wrt the world to body axis_angle,
        then axis_angle should be the - of the body to world axis_angle in robot_state.
        and R should be robot_state.RT, the world to body rotation.
    the magnitude of axis_angle is the angle of rotation.
unrotated_vector is the vector before rotation. 
sets 3x3 Jacobian  */
void Jac_rotated_wrt_axis_angle(Eigen::Ref<Matrix3f> Jacobian, const Vector3f &axis_angle, const Matrix3f &R,
                                const Vector3f &unrotated_vector);

/* Given a vector, calculate its unit vector and the jacobian of the unit vector wrt the vector*/
void jacobian_unitvec_wrt_vec(Matrix3f &jacobian, const Vector3f &vec);

Matrix3f axis_angle_to_R(const Vector3f &axis_angle);

// create a 3x3 matrix such that matrix multiplication raised_cross_matrix(a) * b = cross(a, b)
Matrix3f raised_cross_matrix(const Vector3f &vec);

#endif  // WALKER_ROTATION_HPP
