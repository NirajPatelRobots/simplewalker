# -*- coding: utf-8 -*-
"""
Created March 2021
TODO:
    tests
    I'm sure MotionController_data isn't very pythonic but I refuse to read a style guide
    tests have a wide range s.t. we expect some tracking error fails

@author: Niraj
"""
import numpy as np
import kinematics as kin
from motionPlanner import MotionPlanner
from copy import deepcopy
from os.path import exists
from pickle import dump, load


class MotionController:
    """A motion control calculator.
    use MC.set_plan() to set a new plan
    use MC.calc_controller() to calculate motor commands from angle sensors.
    MC.dt is the time between calc_controller or set_leg_angles calls
    The first time you receive angle data, or any other time you have angle data
        but don't run calc_controller, use MC.set_leg_angles() instead to keep timing."""
    def __init__(self, dt = 1/20):
        """ constructor.
        dt is the time between calc_controller calls"""
        self.plan = MotionPlanner()
        self._rightAngles = np.zeros((3,1))
        self._leftAngles = np.zeros((3,1))
        self._rightVel = np.zeros((3,1))
        self._leftVel = np.zeros((3,1))
        self._ik_Jac = np.zeros((3,3))
        self.dt = dt
        #store data about legs and stuff
        self.params = MotionController_data.load("settings/MotionController.dat")
        if self.params is None:
            self.params = MotionController_data()
        self.plan.createPlan(self.params.body_default_pos, self.params.right_default_pos, self.params.left_default_pos, np.zeros(3))
        
    def calc_controller(self, rightAngles, leftAngles, body_inclination):
        """calculate the control signals to set the motor to.
        Advances the plan by one step.
        Must be called every self.dt seconds for accurate calculation.
        rightAngles and leftAngles are 3x1 arrays of motor angles.
        body_inclination $alpha$ is the inclination of the body
        
        returns (rightCommands, leftCommands)
        each is an array of [theta0_desired, D_theta1, D_theta2]
        where theta0_desired is the desired theta0 angle,
        D_theta1 and D_theta2 are the duty cycle for the motors."""
        self.set_leg_angles(rightAngles, leftAngles)
        right_pos_ref, left_pos_ref, right_vel_ref, left_vel_ref = self._get_leg_target(self.plan)
        right_error, right_vel_error = self._leg_angle_error(self._rightAngles, self._rightVel, right_pos_ref, right_vel_ref, body_inclination)
        left_error, left_vel_error = self._leg_angle_error(self._leftAngles, self._leftVel, left_pos_ref, left_vel_ref, body_inclination)
        right_on_ground = (right_pos_ref[kin.UP] < 1e-3)
        left_on_ground = (left_pos_ref[kin.UP] < 1e-3)
        if (right_pos_ref[kin.UP] < 1e-3):
            if (left_pos_ref[kin.UP] < 1e-3):
                right_on_ground = 0.5
                left_on_ground = 0.5
            else:
                right_on_ground = 1.0
                left_on_ground = 0.0
        else:
            if (left_pos_ref[kin.UP] < 1e-3):
                left_on_ground = 1.0
                right_on_ground = 0.0
            else:
                left_on_ground = 0.0
                right_on_ground = 0.0
        
        rightCommands = np.empty(3)
        leftCommands = np.empty(3)
        rightCommands[0] = self._calc_theta0_ref(right_pos_ref)
        leftCommands[0] = self._calc_theta0_ref(left_pos_ref)
        rightCommands[1] = self._calc_motor1(right_error, right_vel_error, body_inclination, right_on_ground)
        leftCommands[1] = self._calc_motor1(left_error, left_vel_error, body_inclination, left_on_ground)
        rightCommands[2] = self._calc_motor2(self._rightAngles, right_error, right_vel_error, body_inclination, right_on_ground)
        leftCommands[2] = self._calc_motor2(self._leftAngles, left_error, left_vel_error, body_inclination, left_on_ground)
        
        return (rightCommands, leftCommands)
    
    def set_plan(self, plan):
        """ set a new plan for the motion controller to follow.
        the plan is a MotionPlanner object where the first element is current position."""
        self.plan = deepcopy(plan)
        self.plan.advance()
    
    def set_leg_angles(self, rightAngles, leftAngles):
        """processes new angle information from angle sensors.
        rightAngles and leftAngles are 3x1 arrays of motor angles.
        Use this function instead of calc_controller to keep running at 1/dt
            rate when you don't run calc_controller
        """
        rightAngles = rightAngles.reshape(3,1)
        leftAngles = leftAngles.reshape(3,1)
        self._rightVel = (rightAngles - self._rightAngles) / self.dt
        self._leftVel = (leftAngles - self._leftAngles) / self.dt
        self._rightAngles = rightAngles
        self._leftAngles = leftAngles
    
    def _get_leg_target(self, plan):
        """Gets the target position and velocity for the leg from the plan.
        plan is the MotionPlanner, which is advanced to the next point.
        returns targets as (3x1) arrays in robot coordinates
        where positions are from the leg connections to the feet and velocities are derivatives of those.
        # returns (right_pos_ref, left_pos_ref, right_vel_ref, left_vel_ref)."""
        # right_leg_pos = (plan.rightPos[:,0] - (plan.pos[:,0] + self.params.right_leg_conn)).reshape(3,1)
        # left_leg_pos = (plan.leftPos[:,0] - (plan.pos[:,0] + self.params.left_leg_conn)).reshape(3,1)
        right_leg_pos = plan.rightPos[:,0].reshape(3,1)
        left_leg_pos = plan.leftPos[:,0].reshape(3,1)
        right_leg_vel = (plan.rightVel[:,0] - plan.vel[:,0]).reshape(3,1)
        left_leg_vel = (plan.leftVel[:,0] - plan.vel[:,0]).reshape(3,1)
        plan.advance()
        
        return (right_leg_pos, left_leg_pos, right_leg_vel, left_leg_vel)
    
    def _leg_angle_error(self, angle, angVel, pos_ref, vel_ref, body_inclination):
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
        ik_Jac = kin.ik_Jacobian(fk_Jac, angle)
        ik_Jac_dot = (ik_Jac - self._ik_Jac) / self.dt
        self._ik_Jac = ik_Jac
        
        angle_error = ik_Jac @ (pos_ref - pos.reshape(3,1))
        angVel_error = ik_Jac_dot @ (pos_ref - pos.reshape(3,1)) + ik_Jac @ (vel_ref - angVel.reshape(3,1)) #chain rule
        np.clip(angle_error, -np.pi, np.pi, out=angle_error)
        
        return (angle_error, angVel_error)
        
    def _calc_theta0_ref(self, pos_ref):
        """returns (scalar) desired hip sideways angle
        pos_ref is (3x1) position from the leg connection to the foot
        """
        return np.arcsin(pos_ref[kin.RIGHT] / np.linalg.norm(pos_ref))
        
    def _calc_motor1(self, angle_error, vel_error, body_inclination, on_ground):
        """ calculate the duty cycle for motor 1, the hip forwards motor.
        angle_error and vel_error are (3x1) (desired - actual) angles and angular velocities.
        body_inclination is float body inclination
        on_ground is 1.0 if only this foot is on the ground and supporting weight, 0.5 if both are, 0.0 if this foot isn't
        returns float duty cycle between -1 and 1
        """
        DC = (self.params.kp_1 * angle_error[1]
              + self.params.kv_1 * vel_error[1]
              - self.params.kp_alpha * body_inclination * on_ground)
        if DC > 1:
            return 1.0
        if DC < -1:
            return -1.0
        return DC
        
        
    def _calc_motor2(self, angle, angle_error, vel_error, body_inclination, on_ground):
        """ calculate the duty cycle for motor 2, the knee motor.
        angle is the current motor angles
        angle_error and vel_error are (3x1) (desired - actual) angles and angular velocities.
        body_inclination is float body inclination
        returns float duty cycle between -1 and 1
        """
        DC = (self.params.kp_2* angle_error[2]
              + self.params.kv_2 * vel_error[2]
              - self.params.body_weight * kin.THIGH_LENGTH * np.sin( + angle[1]) * on_ground)
        if DC > 1:
            return 1.0
        if DC < -1:
            return -1.0
        return DC
    
class MotionController_data:
    """store data about motion controller assumptions and tuning"""
    def __init__(self):
        self.right_leg_conn = 0.03*kin.RIGHT_DIR
        self.left_leg_conn = -0.03*kin.RIGHT_DIR
        self.body_weight = .5 * 9.81 # [N]
        self.right_default_pos = -0.30*kin.UP_DIR + 0.05*kin.RIGHT_DIR
        self.left_default_pos =  -0.30*kin.UP_DIR - 0.05*kin.RIGHT_DIR
        self.body_default_pos =   0.30*kin.UP_DIR
        self.kp_1 = 5.0 #proportional gain on motor 1
        self.kp_2 = 5.0
        self.kv_1 = 0.0 #velocity gain on motor 1
        self.kv_2 = 0.0
        self.kp_alpha = 2.0
    
    @staticmethod
    def load(filename):
        """static method that returns a MotionController_data object saved at filename"""
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
    MC = MotionController()
    kin.forward_kinematics(np.zeros(3), 0.0, return_Jacobian=True) #compile
    MC.set_leg_angles(np.zeros(3), np.zeros(3))
    MC.params.kv_1 = 0.0 # don't test dynamics so we can use random inputs and not rotate right_default_pos
    MC.params.kv_2 = 0.0
    angles = np.array([[np.pi/2], [-3/8*np.pi], [3/8*np.pi]]) * np.random.random((3,numTries))
    angles += np.array([[0.0], [0.0], [np.pi/4]])
    inclination = (np.random.random((numTries))-0.5) * 0.0 #np.pi/4
    
    rightCommands = np.empty((3,numTries))
    leftCommands = np.empty((3,numTries))
    
    #test calc_controller speed
    startTime = process_time()
    for i in range(numTries):
        rightCommands[:,i], leftCommands[:,i] = MC.calc_controller(angles[:,i], kin.LEFTSCALE * angles[:,i], inclination[i])
    calc_time = process_time() - startTime
    print("calc_controller takes", round(calc_time * 1e6 / numTries), "us")
    
    #test identical left and right results
    if np.all(np.isclose(rightCommands, kin.LEFTSCALE_L * leftCommands)):
        print("Right and left commands were symmetrical")
    else:
        print("FAIL: Right and left commands were not symmetrical")
        
    default_angles = kin.inverse_kinematics(MC.params.right_default_pos, 0.0, theta_est = [0.0, -1.0, 1.0])
    print("Default angles", default_angles, "(rad)", default_angles * 180 / np.pi, "(deg)")
    
    #test leg angle error using inverse_kinematics
    #angles_ref = angles + (np.random.random((3,numTries)) - 0.5) * 0.1
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
            plt.ylabel("Angle 1 error [rad]")
            plt.subplot(2, 1, 2)
            plt.plot(angles[2,:], pred_err[2,:], 'r.', angles[2,:], err[2,:], 'b.')
            plt.grid(True)
            plt.xlabel("Angle 2 [rad]")
            plt.ylabel("Angle 2 error [rad]")
    
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
        t1angles = np.array([angles[0,i], angles[1,i] + 0.01 * rightCommands[1,i], angles[2,i]])
        t2angles = np.array([angles[0,i], angles[1,i], angles[2,i] + 0.01 * rightCommands[2,i]])
        
        startDist = np.sum((MC.params.right_default_pos - kin.forward_kinematics(angles[:,i], inclination[i]))**2)
        t0Dist = np.sum((MC.params.right_default_pos - kin.forward_kinematics(t0angles, inclination[i]))**2)
        t1Dist = np.sum((MC.params.right_default_pos - kin.forward_kinematics(t1angles, inclination[i]))**2)
        t2Dist = np.sum((MC.params.right_default_pos - kin.forward_kinematics(t2angles, inclination[i]))**2)
        
        startSpin = np.sum((default_angles - angles[:,i])**2)
        t0Spin = np.sum((default_angles - t0angles)**2)
        t1Spin = np.sum((default_angles - t1angles)**2)
        t2Spin = np.sum((default_angles - t2angles)**2)
        if t0Dist > startDist:
            t0fails += 1
        if t1Dist > startDist:
            t1fails += 1
        if t2Dist > startDist:
            t2fails += 1
        if t0Spin > startSpin:
            t0Angfails += 1
        if t1Spin > startSpin:
            t1Angfails += 1
        if t2Spin > startSpin:
            t2Angfails += 1
    if t0fails > 0:
        print("FAIL: Angle command 0 failed to improve tracking error", t0fails, "(", t0Angfails, ")/", numTries, "times")
    if t1fails > 0:
        print("FAIL: Angle command 1 failed to improve tracking error", t1fails, "(", t1Angfails, ")/", numTries, "times")
    if t2fails > 0:
        print("FAIL: Angle command 2 failed to improve tracking error", t2fails, "(", t2Angfails, ")/", numTries, "times")
    
    
    # test parameters saving
    params = MotionController_data()
    params.body_weight = np.random.random()
    params.save("settings/testmc.dat")
    newparams = MotionController_data.load("settings/testmc.dat")
    if params.body_weight == newparams.body_weight:
        print("Save test passed")
    else:
        print("FAIL: updated params didn't save")
    
if __name__ == "__main__":
    test()

