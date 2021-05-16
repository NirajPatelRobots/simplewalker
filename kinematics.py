# -*- coding: utf-8 -*-
"""
Library for kinematic calculations for walking robot
TODO:
    fk_Jacobian_dot? complicated, so numerically calculated by motionController instead.

Created Mar 2021
@author: Niraj
"""
import numpy as np
from scipy.optimize import minimize
from numba import njit

SHIN_LENGTH = 0.2 # [m]
THIGH_LENGTH = 0.2 # [m]

# array indexes
RIGHT = 0
FORWARD = 1
UP = 2
HORIZONTAL = (FORWARD, RIGHT)
RIGHT_DIR = np.array([1,0,0])
FORWARD_DIR = np.array([0,1,0])
UP_DIR = np.array([0,0,1])

LEFTSCALE = np.ones(3)
LEFTSCALE[RIGHT] *= -1 #multiply by this to change right to left

def forward_kinematics(theta, body_inclination, return_Jacobian=False):
    """given motor angles, find the expected position of the foot.
    theta[0] is the hip's angle, zero when the leg is pointing down, positive outwards from the side of the body.
    theta[1] is the hip's angle, zero when the leg is pointing down, positive forwards
    theta[2] is the knee angle, zero when the knee is straight
    body_inclination $alpha$ is how much the body is tilted relative to the plane of the ground, positive upwards.
    return_Jacobian: True to return (p_leg, fk_Jacobian), False (default) returns p_leg.
        See kinematics.fk_Jacobian
    
    returns relative leg position p_leg 3-array in robot coordinates"""
    phi1 = body_inclination + theta[1]
    phi2 = phi1 + theta[2]
    phi = np.array([theta[0], phi1, phi2])
    sines = np.sin(phi).reshape(3)
    cosines = np.cos(phi).reshape(3)
    p_leg = np.empty(3)
    p_leg[FORWARD] = (THIGH_LENGTH * sines[1] + SHIN_LENGTH * sines[2]) * cosines[0]
    p_leg[RIGHT] = (THIGH_LENGTH * cosines[1] + SHIN_LENGTH * cosines[2]) * sines[0]
    p_leg[UP] = -(THIGH_LENGTH * cosines[1] + SHIN_LENGTH * cosines[2]) * cosines[0]
    if return_Jacobian:
        Jac = _fk_Jac_physTrig(sines, cosines)
        return p_leg, Jac
    return p_leg

def fk_Jacobian(theta, body_inclination):
    """given motor angles, find the Jacobian d(p_leg)/d(motor_angle)
        where p_leg is relative leg position array of [forward, out, up].
    theta[0] is the hip outward motor angle, zero when the leg is pointing down, positive outwards from the side of the body.
    theta[1] is the hip forward motor angle, zero when the leg is pointing down, positive forwards
    theta[2] is the knee angle, zero when the knee is straight, positive forwards
    down is towards the ground, forward and out are body orientation, all 3 are perpendicular to each other
    
    using both fk_Jacobian() and forward_kinematics() is slower than forward_kinematics(return_Jacobian=True)
    
    returns Jacobian as 3x3 matrix"""
    phi1 = body_inclination + theta[1]
    phi2 = phi1 + theta[2]
    phi = np.array([theta[0], phi1, phi2])
    return _fk_Jac_physTrig(np.sin(phi), np.cos(phi))

def fk_Jacobian_dot(theta, omega):
    """given leg angles and angular velocity, find the time derivative of the Jacobian returned by fk_Jacobian.
    theta[0] is the hip outward motor angle, zero when the leg is pointing down, positive outwards from the side of the body.
    theta[1] is the hip forward motor angle, zero when the leg is pointing down, positive forwards
    theta[2] is the knee angle, zero when the knee is straight, positive forwards
    
    returns 3x3 time derivative of Jacobian matrix"""
    #TODO? complicated, so numerically calculated by motionController instead.
    
def inverse_kinematics(p_leg, body_inclination, theta_est = None, ignore_failure = False):
    """given the position of the leg, find the desired motor angles.
    Calls a solver, so slower than forward_kinematics.
    p_leg is relative leg position array of [forward, out, up]
    theta_est is initial guess. default is [0, 0, 0]
    if the solver fails and ignore_failure is False, the function will return None
    
    returns array [theta0, theta1, theta2]"""
    def f(theta, body_inclination, p_desired):
        p_leg, Jac = forward_kinematics(theta, body_inclination, return_Jacobian=True)
        disp = p_leg - p_desired
        return np.sum(disp**2), disp @ Jac
    if theta_est is None:
        theta_est = np.zeros(3)
    
    #ret = root(f, theta_est, args=(body_inclination, p_leg), jac=True)
    ret = minimize(f, theta_est, args=(body_inclination, p_leg), jac=True, tol=1e-6)
    #angles = (ret.x + np.pi) % (2 * np.pi) - np.pi # wrap -pi to pi
    if ret.success or ignore_failure:
        return ret.x
    #print(ret.message)
    return None
    

""" --------------------- internal use functions ------------------ """
@njit
def _fk_Jac_physTrig(sines, cosines):
    """given ratios of physical coordinates, find d(p_leg)/d(motor_angle)
    theta0 is the motor angle outwards
    phi1 is angle between thigh and down in the plane of the leg
    phi2 is the angle between shin and down in the plane of the leg
    sines and cosines are sin and cos of [theta0, phi1, phi2]
    
    returns Jacobian as a 3x3 matrix
    """
    Jac = np.zeros((3,3))
    Jac[FORWARD,0] = -(sines[1] + sines[2]) * sines[0]
    Jac[FORWARD,1] = (cosines[1] + cosines[2]) * cosines[0]
    Jac[FORWARD,2] = cosines[2] * cosines[0]
    Jac[RIGHT,0] = (cosines[1] + cosines[2]) * cosines[0]
    Jac[RIGHT,1] = -(sines[1] + sines[2]) * sines[0]
    Jac[RIGHT,2] = -sines[2] * sines[0]
    Jac[UP,0] = (cosines[1] + cosines[2]) * sines[0]
    Jac[UP,1] = (sines[1] + sines[2]) * cosines[0]
    Jac[UP,2] = sines[2] * cosines[0]
    return Jac
    

def test():
    import time
    rng = np.random.default_rng()
    numTries = 5000
    # thetas is [theta0 theta1 theta2 alpha] for each try
    thetas = rng.uniform(low=-np.pi/2, high=np.pi/2, size=(4,numTries))
    fk_results = np.empty((3,numTries))
    Jac_results = np.empty((3,3,numTries))
    fk_Jac_results = np.empty((3,3,numTries))
    ik_thetas = thetas[:3,:] + rng.uniform(low=-0.1, high=0.1, size=(3,numTries))
    ik_results = np.empty((3,numTries))
    ik_fails = np.zeros((numTries), dtype = int)
    
    forward_kinematics(np.zeros(3), 0.0)
    fk_Jacobian(np.zeros(3), 0.0) #compile
    
    startTime = time.perf_counter()
    for i in range(numTries):
        fk_results[:,i] = forward_kinematics(thetas[:3,i], thetas[3,i])
    fk_time = time.perf_counter() - startTime
    print("forward_kinematics takes", round(fk_time * 1e6 / numTries, 3), "us")
    
    startTime = time.perf_counter()
    for i in range(numTries):
        Jac_results[:,:,i] = fk_Jacobian(thetas[:3,i], thetas[3,i])
    Jac_time = time.perf_counter() - startTime
    print("fk_Jacobian takes", round(Jac_time * 1e6 / numTries, 3), "us")
    
    startTime = time.perf_counter()
    for i in range(numTries):
        _, fk_Jac_results[:,:,i] = forward_kinematics(thetas[:3,i], thetas[3,i], return_Jacobian=True)
    fk_Jac_time = time.perf_counter() - startTime
    print("forward_kinematics with Jacobian takes", round(fk_Jac_time * 1e6 / numTries, 3), "us")
    
    if not np.all(np.isclose(Jac_results, fk_Jac_results)):
        print("FAIL: forward_kinematics and fk_Jacobian yielded different values")
    
    startTime = time.perf_counter()
    for i in range(numTries):
        ret = inverse_kinematics(fk_results[:,i], thetas[3,i], theta_est = ik_thetas[:3,i])
        if ret is None:
            ik_fails[i] = 1
        else:
            ik_results[:,i] = ret
    ik_time = time.perf_counter() - startTime
    print("inverse_kinematics takes", round(ik_time * 1e6 / numTries, 3), "us with close estimate")
    print(np.sum(ik_fails), "/", numTries, "inverse_kinematics optimizer failures")
    
    #BUG: there are two ik values, inverse_kinematics could pick the wrong one
    failnum = 0
    for i in range(numTries):
        if not (np.all(np.isclose(forward_kinematics(ik_results[:,i], thetas[3,i]), fk_results[:,i], atol=0.001)) or ik_fails[i]):
            #print("FAIL: inverse_kinematics on forward_kinematics yielded different values at", i)
            print(ik_results[:,i], " ||| ", thetas[:3,i])
            print("\t", forward_kinematics(ik_results[:,i], thetas[3,i]), " ||| ", fk_results[:,i])
            failnum += 1
    print(failnum, "inverse_kinematics incorrect results")
    
        
    
if __name__ == "__main__":
    test()
    
    