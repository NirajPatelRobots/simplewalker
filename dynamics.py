# -*- coding: utf-8 -*-
"""
Created Sept 2021 based on motionController.py
runtime:
    8% is ik_Jac_dot chain rule term from velocity error calculation
    27% is ik_Jac and ik_Jac_dot calculations
    92% leg.calculate
TODO:
    body weight

@author: Niraj
"""
import numpy as np
import kinematics as kin
from motionPlanner import MotionPlanner
from copy import deepcopy

class Dynamics:
    """class that contains multiple LegDynamics.
    Interprets the plan, calculates legs"""
    def __init__(self, dt, learned_params = None):
        """dt is dt
        learned_params is None or an array of [right hip, right knee, left hip, left knee]
            where each element is a dict of learned motor parameters or None"""
        self.plan = MotionPlanner()
        self.dt = dt
        if learned_params is None:
            learned_params = [None, None, None, None]
        self.right = LegDynamics(dt, learned_params[0], learned_params[1])
        self.left = LegDynamics(dt, learned_params[2], learned_params[3])
        self.right_leg_conn = 0.03*kin.RIGHT_DIR
        self.left_leg_conn = -0.03*kin.RIGHT_DIR
        
    def calculate(self, right_angle, right_angVel, left_angle, left_angVel, body_inclination):
        """find the leg reference angles and angular velocity from the next step in the plan
        takes leg angles and angular velocity as (3x1) and body_inclination"""
        right_pos_ref = (self.plan.rightPos[:,0] - (self.plan.pos[:,0] + self.right_leg_conn)).reshape(3,1)
        left_pos_ref = (self.plan.leftPos[:,0] - (self.plan.pos[:,0] + self.left_leg_conn)).reshape(3,1)
        right_vel_ref = (self.plan.rightVel[:,0:1] - self.plan.vel[:,0:1])
        left_vel_ref = (self.plan.leftVel[:,0:1] - self.plan.vel[:,0:1])
        self.right.calculate(right_angle, right_angVel, right_pos_ref, right_vel_ref, body_inclination)
        self.left.calculate(left_angle, left_angVel, left_pos_ref, left_vel_ref, body_inclination)
        self.plan.advance()
        
    def setPlan(self, plan):
        """set a MotionPlanner plan which starts with current position.
        This plan is the cartesian paths the body and feet should follow."""
        self.plan = deepcopy(plan)
        self.plan.advance()
        

class LegDynamics:
    """calculate dynamics for the use of motor controller
    use LegDynamics.set_plan() to set a new plan
    use LegDynamics.calculate() to calculate motor commands from angle sensors.
    then access LegDynamics.pos, LegDynamics.angle_ref, LegDynamics.angvel_ref, LegDynamics.load_torque"""
    def __init__(self, dt, hip_learned_params = None, knee_learned_params = None):
        """ constructor.
        dt is time difference between function calls
        learned_params is the learned parameters for the leg
        """
        self.dt = dt
        self.ik_Jac = np.identity(3)
        self.pos = np.zeros((3,1))
        self.angle_ref = np.zeros(3)
        self.angvel_ref = np.zeros(3)
        self.grounded = 0.
        self.load_torque = np.zeros(3)
        if not hip_learned_params is None and "leg_weight_1" in hip_learned_params.keys():
            self.leg_weight_1 = hip_learned_params["leg_weight_1"]
        else:
            self.leg_weight_1 = 0.
        if not knee_learned_params is None and "leg_weight_2" in knee_learned_params.keys():
            self.leg_weight_2 = knee_learned_params["leg_weight_2"]
        else:
            self.leg_weight_2 = 0.
        if not hip_learned_params is None and "leg_weight_12" in hip_learned_params.keys():
            self.leg_weight_12 = hip_learned_params["leg_weight_12"]
        else:
            self.leg_weight_12 = 0.
        
        
    def calculate(self, angle, angVel, pos_ref, vel_ref, body_inclination, dt=None):
        """calculate the reference angle and load torque on the motors
        angle is 3x1 array of motor angles 
        pos_ref is desired 3D cartesian position
        alt_pos_ref is desired other leg position
        body_inclination $alpha$ is the inclination of the body
        dt is optional temporary change in timestep. default is object parameter."""
        if dt is None:
            dt = self.dt
        self.pos, fk_Jac = kin.forward_kinematics(angle, body_inclination, return_Jacobian=True)
        new_ik_Jac = kin.ik_Jacobian(fk_Jac, angle)
        ik_Jac_dot = (new_ik_Jac - self.ik_Jac) / self.dt
        self.ik_Jac[:] = new_ik_Jac
        
        angle_error = self.ik_Jac @ (pos_ref - self.pos.reshape(3,1))
        angVel_error = self.ik_Jac @ (vel_ref - angVel.reshape(3,1)) + ik_Jac_dot @ (pos_ref - self.pos.reshape(3,1)) #chain rule
        np.clip(angle_error, -np.pi, np.pi, out=angle_error)
        
        self.angle_ref[0] = np.arcsin(pos_ref[kin.RIGHT] / np.linalg.norm(pos_ref))
        self.angle_ref[1:] = angle[1:] + angle_error[1:].reshape(-1)
        self.angvel_ref = angVel + angVel_error
        self.load_torque[1] = -self.leg_weight_1 * np.sin(body_inclination + angle[1]) \
                            + self.leg_weight_12 * np.sin(body_inclination + angle[1] + angle[2])
        self.load_torque[2] = -self.leg_weight_2 * np.sin(body_inclination + angle[1] + angle[2])
        
        
def test():
    numTries = 1000
    makePlot = True
    from time import process_time
    dyn = Dynamics(0.05)
    kin.forward_kinematics(np.zeros(3), 0.0, return_Jacobian=True) #compile
    
    default_p_leg = dyn.plan.rightPos[:,0] - (dyn.plan.pos[:,0] + dyn.right_leg_conn)
    default_angles = kin.inverse_kinematics(default_p_leg, 0.0, theta_est = [0.0, -1.0, 1.0])
    if not default_angles is None:
        print("Default angles", default_angles, "(rad)", default_angles * 180 / np.pi, "(deg)")
    else:
        default_angles = np.array([0.066568, -0.720212, 1.440426])
    angle_variation = [[0.4], [-1.], [1.]]
    angles = np.array(angle_variation) * np.random.random((3,numTries))
    angles += np.array(default_angles.reshape(3,1) - np.array(angle_variation)/2)
    inclination = (np.random.random((numTries))-0.5) * 0.0 #TODO vary inclination
    
    right_angle_ref = np.empty((3,numTries))
    left_angle_ref = np.empty((3,numTries))
    zero = np.zeros((3,1))
    
    #test calculate speed
    startTime = process_time()
    for i in range(numTries):
        dyn.calculate(angles[:,i], zero, kin.LEFTSCALE * angles[:,i], zero, inclination[i])
        right_angle_ref[:,i] = dyn.right.angle_ref
        left_angle_ref[:,i] = dyn.left.angle_ref
    calc_time = process_time() - startTime
    print("calculate takes", round(calc_time * 1e6 / numTries), "us")
    
    #test identical left and right results
    if np.all(np.isclose(right_angle_ref, kin.LEFTSCALE_L * left_angle_ref)):
        print("Right and left commands were symmetrical")
    else:
        print("FAIL: Right and left commands were not symmetrical")
    
    #test leg angle prediction
    err = np.empty((3, numTries))
    for i in range(numTries):
        err[:, i] = default_angles - right_angle_ref[:,i]
    if makePlot:
        try:
            import matplotlib.pyplot as plt
        except:
            pass
        else:
            fig = plt.figure(1)
            fig.clf()
            ax = fig.add_subplot(projection = '3d')
            #plt.subplot(2, 1, 1)
            ax.scatter(angles[1,:], angles[2,:], zs=err[1,:])
            plt.grid(True)
            plt.xlabel("Angle 1 [rad]")
            plt.ylabel("Angle 2 [rad]")
            ax.set_zlabel("Angle 1 error (capped) [rad]")
            fig = plt.figure(2)
            fig.clf()
            ax = fig.add_subplot(projection = '3d')
            ax.scatter(angles[1,:], angles[2,:], zs=err[2,:])
            plt.grid(True)
            plt.xlabel("Angle 1 [rad]")
            plt.ylabel("Angle 2 [rad]")
            ax.set_zlabel("Angle 2 error (capped) [rad]")
    
if __name__ == "__main__":
    test()

