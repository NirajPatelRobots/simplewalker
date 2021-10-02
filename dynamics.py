# -*- coding: utf-8 -*-
"""
Created Sept 2021 based on motionController.py
TODO:
    I'm sure Dynamics_params isn't very pythonic but I refuse to read a style guide
    tests have some tracking error fails
    resolve planner._hstar and Dynamics.params.body_default_pos
    split into one for each leg? anything it needs to know about other legs it can get from plan? Yes

@author: Niraj
"""
import numpy as np
import kinematics as kin
from motionPlanner import MotionPlanner
from copy import deepcopy
from os.path import exists
from pickle import dump, load


class Dynamics:
    """calculate dynamics for the use of motor controller
    Dyn = Dynamics(dt) where dt is time between calls
    use Dynamics.set_plan() to set a new plan
    use Dynamics.calculate() to calculate motor commands from angle sensors.
    dt is time difference between function calls
    then access Dyn.right_angle_ref, self.right_grounded, self.right_torque and equivalent left"""
    def __init__(self, dt):
        """ constructor."""
        self.plan = MotionPlanner()
        self._rightAngles = np.zeros((3,1))
        self._leftAngles = np.zeros((3,1))
        self._rightVel = np.zeros((3,1))
        self._leftVel = np.zeros((3,1))
        self._right_ik_Jac = np.identity(3)
        self._left_ik_Jac = np.identity(3)
        self.right_angle_ref = np.zeros(3)
        self.left_angle_ref = np.zeros(3)
        self.right_angvel_ref = np.zeros(3)
        self.left_angvel_ref = np.zeros(3)
        self.right_grounded = 0.
        self.left_grounded = 0.
        self.dt = dt
        #store data about legs and stuff
        self.params = Dynamics_params.load("settings/Dynamics.dat")
        if self.params is None:
            self.params = Dynamics_params()
        self.plan.createPlan(self.params.body_default_pos, self.params.right_default_pos, self.params.left_default_pos, np.zeros(3))
        
    def calculate(self, rightAngles, leftAngles, rightVel, leftVel, body_inclination):
        """calculate the control signals to set the motor to.
        Advances the plan by one step.
        rightAngles and leftAngles are 3x1 arrays of motor angles, rightVel and leftVel are angular velocities
        body_inclination $alpha$ is the inclination of the body"""
        self.set_leg_angles(rightAngles, leftAngles, rightVel, leftVel)
        right_pos_ref, left_pos_ref, right_vel_ref, left_vel_ref = self._get_leg_target(self.plan)
        right_error, right_vel_error = self._leg_angle_error(self._rightAngles, self._rightVel, right_pos_ref, right_vel_ref, self._right_ik_Jac, body_inclination)
        left_error, left_vel_error = self._leg_angle_error(self._leftAngles, self._leftVel, left_pos_ref, left_vel_ref, self._left_ik_Jac, body_inclination)
        self.right_grounded, self.left_grounded =  self._get_grounded(self.plan)
        self.right_angle_ref[0] = self._calc_theta0_ref(right_pos_ref)
        self.left_angle_ref[0] = self._calc_theta0_ref(left_pos_ref)
        self.right_angle_ref[1:] = self._rightAngles[1:] + right_error[1:]
        self.left_angle_ref[1:] = self._leftAngles[1:] + left_error[1:]
        self.right_angvel_ref = rightVel + right_vel_error
        self.left_angvel_ref = leftVel + left_vel_error
    
    def set_plan(self, plan):
        """ set a new plan for the motion controller to follow.
        the plan is a MotionPlanner object where the first element is current position."""
        self.plan = deepcopy(plan)
        self.plan.advance()
    
    def set_leg_angles(self, rightAngles, leftAngles, rightVel, leftVel):
        """processes new angle information from angle sensors.
        rightAngles and leftAngles are 3x1 arrays of motor angles.
        Use this function instead of calculate to keep running at 1/dt
            rate when you don't run calculate"""
        rightAngles = rightAngles.reshape(3,1)
        leftAngles = leftAngles.reshape(3,1)
        self._rightVel = rightVel.reshape(3,1)
        self._leftVel = leftVel.reshape(3,1)
        self._rightAngles = rightAngles
        self._leftAngles = leftAngles
    
    def _get_leg_target(self, plan):
        """Gets the target position and velocity for the leg from the plan.
        plan is the MotionPlanner, which is advanced to the next point.
        returns targets as (3x1) arrays in robot coordinates
        where positions are from the leg connections to the feet and velocities are derivatives of those.
        # returns (right_pos_ref, left_pos_ref, right_vel_ref, left_vel_ref)."""
        right_leg_pos = (plan.rightPos[:,0] - (plan.pos[:,0] + self.params.right_leg_conn)).reshape(3,1)
        left_leg_pos = (plan.leftPos[:,0] - (plan.pos[:,0] + self.params.left_leg_conn)).reshape(3,1)
        right_leg_vel = (plan.rightVel[:,0] - plan.vel[:,0]).reshape(3,1)
        left_leg_vel = (plan.leftVel[:,0] - plan.vel[:,0]).reshape(3,1)
        plan.advance()
        
        return (right_leg_pos, left_leg_pos, right_leg_vel, left_leg_vel)
    
    def _leg_angle_error(self, angle, angVel, pos_ref, vel_ref, body_inclination, ik_Jac):
        """ Calculates the motor angle error between current and desired position.
        Parameters
        angle : (3x1) array
            the current motor angles of the leg.
        angVel : (3x1) array
            the current motor angular velocities of the leg.
        pos_ref : (3x1) array
            Desired leg position from leg connection to foot, in robot coordinates.
        vel_ref : (3x1) array
            derivative of pos_ref at this point.
        body_inclination : (3x1) array
            inclination of the body

        Returns
        angle_error : (3x1) array
            estimated difference between current and desired motor angle
        angVel_error : (3x1) array
            estimated difference between current and desired motor angular velocity
        """
        pos, fk_Jac = kin.forward_kinematics(angle, body_inclination, return_Jacobian=True)
        new_ik_Jac = kin.ik_Jacobian(fk_Jac, angle)
        ik_Jac_dot = (new_ik_Jac - ik_Jac) / self.dt
        ik_Jac[:] = new_ik_Jac
        
        angle_error = ik_Jac @ (pos_ref - pos.reshape(3,1))
        angVel_error = ik_Jac_dot @ (pos_ref - pos.reshape(3,1)) + ik_Jac @ (vel_ref - angVel.reshape(3,1)) #chain rule
        np.clip(angle_error, -np.pi, np.pi, out=angle_error)
        
        return (angle_error, angVel_error)
    
    def _get_grounded(self, plan):
        """given the positions of the feet, return their groundedness.
        right_pos and left_pos are global foot positions (not leg vectors).
        0 if feet is not on ground, 0.5 if both feet on ground, 1 if only foot on ground.
        returns floats right_ground_contact, left_ground_contact
        """
        rightHeight = plan.rightPos[kin.UP,0]
        leftHeight = plan.leftPos[kin.UP,0]
        
        if (rightHeight < 1e-3):
            if (leftHeight < 1e-3):
                right_ground_contact = 0.5
                left_ground_contact = 0.5
                #print("Both")
            else:
                right_ground_contact = 1.0
                left_ground_contact = 0.0
        else:
            if (leftHeight < 1e-3):
                left_ground_contact = 1.0
                right_ground_contact = 0.0
            else:
                left_ground_contact = 0.0
                right_ground_contact = 0.0
        right_ground_contact = 0.0 #TODO remove
        left_ground_contact = 0.0
        return right_ground_contact, left_ground_contact
        
    def _calc_theta0_ref(self, pos_ref):
        """returns (scalar) desired hip sideways angle [rad]
        pos_ref is (3x1) position from the leg connection to the foot"""
        return np.arcsin(pos_ref[kin.RIGHT] / np.linalg.norm(pos_ref))
    
class Dynamics_params:
    """store data about dynamics assumptions and measured values"""
    def __init__(self):
        self.right_leg_conn = 0.03*kin.RIGHT_DIR #body coordinate to leg 
        self.left_leg_conn = -0.03*kin.RIGHT_DIR
        self.right_default_pos = 0.05*kin.RIGHT_DIR
        self.left_default_pos = -0.05*kin.RIGHT_DIR
        self.body_default_pos =  0.30*kin.UP_DIR
    
    @property
    def right_default_leg(self):
        return self.right_default_pos - (self.body_default_pos + self.right_leg_conn)
    @property
    def left_default_leg(self):
        return self.left_default_pos - (self.body_default_pos + self.left_leg_conn)
    
    @staticmethod
    def load(filename):
        """static method that returns a Dynamics_params object saved at filename"""
        if exists(filename):
            with open(filename, "rb") as file:
                return load(file)
        else:
            return None
    
    def save(self, filename):
        with open(filename, "wb") as file:
            dump(self, file, protocol=0)
        
        
def test():
    numTries = 1000
    makePlot = True
    from time import process_time
    MC = Dynamics()
    kin.forward_kinematics(np.zeros(3), 0.0, return_Jacobian=True) #compile
    # lift into the air so it doesn't react to ground forces
    MC.plan.createPlan(MC.params.body_default_pos + 0.01*kin.UP_DIR, MC.params.right_default_pos + 0.01*kin.UP_DIR,
                       MC.params.left_default_pos + 0.01*kin.UP_DIR, np.zeros(3))
    MC.set_leg_angles(np.zeros(3), np.zeros(3))
    MC.params.kv_1 = 0.0 # don't test dynamics so we can use random inputs and not rotate right_default_pos
    MC.params.kv_2 = 0.0
    
    default_angles = kin.inverse_kinematics(MC.params.right_default_leg, 0.0, theta_est = [0.0, -1.0, 1.0])
    if not default_angles is None:
        print("Default angles", default_angles, "(rad)", default_angles * 180 / np.pi, "(deg)")
    else:
        default_angles = np.array([0.16514779, -0.70694972, 1.41390019])
    angle_variation = [[0.01], [-0.1], [0.1]]
    angles = np.array(angle_variation) * np.random.random((3,numTries))
    angles += np.array(default_angles.reshape(3,1) - np.array(angle_variation)/2)
    inclination = (np.random.random((numTries))-0.5) * 0.0 #np.pi/4
    
    rightCommands = np.empty((3,numTries))
    leftCommands = np.empty((3,numTries))
    
    #test calculate speed
    startTime = process_time()
    for i in range(numTries):
        rightCommands[:,i], leftCommands[:,i] = MC.calculate(angles[:,i], kin.LEFTSCALE * angles[:,i], inclination[i])
    calc_time = process_time() - startTime
    print("calculate takes", round(calc_time * 1e6 / numTries), "us")
    
    #test identical left and right results
    if np.all(np.isclose(rightCommands, kin.LEFTSCALE_L * leftCommands)):
        print("Right and left commands were symmetrical")
    else:
        print("FAIL: Right and left commands were not symmetrical")
    
    #test leg angle error
    pred_err = np.empty((3, numTries))
    err = np.empty((3, numTries))
    for i in range(numTries):
        pred_err[:, i:i+1], _ = MC._leg_angle_error(angles[:,i], np.zeros((3,1)), MC.params.right_default_pos.reshape(3,1), np.zeros((3,1)), inclination[i])
        err[:, i] = default_angles - angles[:,i]
    if makePlot:
        try:
            import matplotlib.pyplot as plt
        except:
            pass
        else:
            fig = plt.figure(1)
            fig.clf()
            plt.subplot(2, 1, 1)
            plt.plot(angles[1,:], pred_err[1,:], 'r.', angles[1,:], err[1,:], 'b.')
            plt.grid(True)
            plt.xlabel("Angle 1 [rad]")
            plt.ylabel("Angle 1 error (capped) [rad]")
            plt.subplot(2, 1, 2)
            plt.plot(angles[2,:], pred_err[2,:], 'r.', angles[2,:], err[2,:], 'b.')
            plt.grid(True)
            plt.xlabel("Angle 2 [rad]")
            plt.ylabel("Angle 2 error (capped) [rad]")
    
    # test calc theta0, calc theta1, calc theta2
    # move using each result, check you got closer to goal
    resultAngles = np.empty((3, numTries))
    resultAngles[0,:] = rightCommands[0,:]
    resultAngles[1:,:] = angles[1:,:] + 0.01 * rightCommands[1:,:]
    t0fails = 0
    t1fails = 0
    t2fails = 0
    t0Angfails = 0
    t1Angfails = 0
    t2Angfails = 0
    for i in range(numTries):
        t0angles = np.array([rightCommands[0,i], angles[1,i], angles[2,i]])
        # t1angles = np.array([angles[0,i], angles[1,i] + 0.01 * rightCommands[1,i], angles[2,i]])
        # t2angles = np.array([angles[0,i], angles[1,i], angles[2,i] + 0.01 * rightCommands[2,i]])
        t1angles = np.array([rightCommands[0,i], angles[1,i] + 0.01 * rightCommands[1,i], angles[2,i]])
        t2angles = np.array([rightCommands[0,i], angles[1,i], angles[2,i] + 0.01 * rightCommands[2,i]])
        
        startDist = np.sum((MC.params.right_default_leg - kin.forward_kinematics(angles[:,i], inclination[i]))**2)
        t0Dist = np.sum((MC.params.right_default_leg - kin.forward_kinematics(t0angles, inclination[i]))**2)
        t1Dist = np.sum((MC.params.right_default_leg - kin.forward_kinematics(t1angles, inclination[i]))**2)
        t2Dist = np.sum((MC.params.right_default_leg - kin.forward_kinematics(t2angles, inclination[i]))**2)
        
        startSpin = np.sum((default_angles - angles[:,i])**2)
        t0Spin = np.sum((default_angles - t0angles)**2)
        t1Spin = np.sum((default_angles - t1angles)**2)
        t2Spin = np.sum((default_angles - t2angles)**2)
        if t0Dist > startDist:
            t0fails += 1
        # if t1Dist > startDist:
        #     t1fails += 1
        # if t2Dist > startDist:
        #     t2fails += 1
        if t1Dist > t0Dist:
            t1fails += 1
        if t2Dist > t0Dist:
            t2fails += 1
        if t0Spin > startSpin:
            t0Angfails += 1
        # if t1Spin > startSpin:
        #     t1Angfails += 1
        # if t2Spin > startSpin:
        #     t2Angfails += 1
        if t1Spin > t0Spin:
            t1Angfails += 1
        if t2Spin > t0Spin:
            t2Angfails += 1
    if t0fails > 0:
        print("Angle command 0 failed to improve tracking error", t0fails, "cartesian (", t0Angfails, "leg angle)/", numTries, "times")
    if t1fails > 0:
        print("Angle command 1 failed to improve tracking error", t1fails, "cartesian (", t1Angfails, "leg angle)/", numTries, "times")
    if t2fails > 0:
        print("Angle command 2 failed to improve tracking error", t2fails, "cartesian (", t2Angfails, "leg angle)/", numTries, "times")
    
    
    # test parameters saving
    params = Dynamics_params()
    params.body_weight = np.random.random()
    params.save("settings/testmc.dat")
    newparams = Dynamics_params.load("settings/testmc.dat")
    if params.body_weight == newparams.body_weight:
        print("Save test passed")
    else:
        print("FAIL: updated params didn't save")
    
if __name__ == "__main__":
    test()

