# -*- coding: utf-8 -*-
"""
Motion planning for walking robot
TODO:
    allow for not always on_ground
    plans where to step next
    other behaviors like sway back + forth

Created Mar 2021
@author: Niraj
"""
import numpy as np
from kinematics import UP, LEFTSCALE, HORIZONTAL

class MotionPlanner:
    """Class for a motion planner.
    Use MotionPlanner.createPlan to create a plan.
    Once a plan is created, MotionPlanner.pos is the 3xnumSteps array of positions,
    MotionPlanner.vel is the 3xnumSteps array of velocities.
    same with MP.rightPos, rightVel, leftPos, and leftVel for the leg positions in robot coordinates.
    MotionPlanner.numSteps is the number of points in the plan.
    MotionPlanner.planTime is the length of the plan in seconds."""
    _pos : np.ndarray
    _vel : np.ndarray
    _leftPos : np.ndarray
    _rightPos : np.ndarray
    _leftVel : np.ndarray
    _rightVel : np.ndarray
    _numSteps : int
    _tf : float
    _Jstar : float
    _astar : float
    _hstar : float
    dt : float
    def __init__(self, horizJerk=2.0, heightAccel=2.0, targetHeight=0.3, dt=1/20):
        """horizJerk is the constant Jerk for the horizontal movement.
        heightAccel is constant acceleration upwards or downwards towards target height
        targetHeight [m] is the desired body height
        dt [s] is the time between points in the plan
        """
        self._pos = np.empty((3,0))
        self._vel = np.empty((3,0))
        self._leftPos = np.empty((3,0))
        self._rightPos = np.empty((3,0))
        self._leftVel = np.empty((3,0))
        self._rightVel = np.empty((3,0))
        self._numSteps = 0
        self._tf = 0.0
        self._Jstar = horizJerk
        self._astar = heightAccel
        self._hstar = targetHeight
        self.dt = dt

    def createPlan(self, curr_pos, right_leg_pos, left_leg_pos, curr_vel):
        """ creates a plan based on current position and velocity.
        curr_pos, curr_vel, right_leg_pos and left_leg_pos are 3-arrays of floats for leg position
        The first elements of those arrays are the current position and velocity."""
        pos, vel, h_steps, h_time = self._horizontal_body_plan(curr_pos, curr_vel)
        height, vvel, v_steps, v_time = self._vertical_body_plan(curr_pos[UP])
        
        if h_steps <= 2 and v_steps <= 2: # screen out plans smaller than 1 additional point
            self._pos = curr_pos.reshape((3,1))
            self._vel = np.zeros((3,1))
            self._numSteps = 1
            self._tf = 0.0
        else:
            if v_steps > h_steps: #hang out at last horizontal position
                pos = np.hstack((pos, np.ones((3,v_steps - h_steps)) * pos[:,-1].reshape(3,1)))
                vel = np.hstack((vel, np.zeros((3,v_steps - h_steps))))
                self._numSteps = v_steps
                self._tf = v_time
            pos[UP,:v_steps] = height
            vel[UP,:v_steps] = vvel
            if h_steps >= v_steps:
                pos[UP,v_steps:] = self._hstar
                vel[UP,v_steps:] = 0.0
                self._numSteps = h_steps
                self._tf = h_time
            self._pos = pos
            self._vel = vel
        self._rightPos, self._rightVel = self._leg_plan(right_leg_pos, self._pos, self._vel, self._numSteps)
        self._leftPos, self._leftVel = self._leg_plan(left_leg_pos, self._pos, self._vel, self._numSteps)
    
    def advance(self):
        """remove the current timestep, advancing so that the former second step is now the first step.
        Will not remove the last point in a plan."""
        if self._numSteps > 1:
            self._pos = self._pos[:,1:]
            self._vel = self._vel[:,1:]
            self._leftPos = self._leftPos[:,1:]
            self._rightPos = self._rightPos[:,1:]
            self._leftVel = self._leftVel[:,1:]
            self._rightVel = self._rightVel[:,1:]
            self._numSteps -= 1
            self._tf = max(self._tf - self.dt, 0.0)
        else:
            self._tf = max(self._tf - self.dt, 0.0)
        
        
    def _horizontal_body_plan(self, pos, vel):
        """Create the horizontal part of a plan.
        pos and vel are both 3-arrays of floats.
        returns planPos, planVel, numSteps where numSteps is the number of timesteps the plan will take,
        planPos and planVel are 3xnumSteps arrays of position and velocity points.
        planPos[UP] and planVel[UP] are undefined because this method only plans the horizontal part."""
        #uses constant Jerk model
        vel = np.copy(vel)
        vel[UP] = 0.
        speed = np.linalg.norm(vel)
        if speed < 1e-15:
            direction = np.zeros(3)
        else:
            direction = vel / speed
        t_finish = np.sqrt(2 * speed / self._Jstar) # time to finish horizontal plan
        numSteps = int(np.ceil(t_finish / self.dt))
        planPos = np.empty((3,numSteps+1))
        planVel = np.empty((3,numSteps+1))
        posOffset = pos + self._Jstar / 6 * t_finish**3 * direction
        for i in range(numSteps):
            planPos[:,i] = posOffset + self._Jstar / 6 * (self.dt * i - t_finish)**3 * direction
            planVel[:,i] = self._Jstar / 2 * (self.dt * i - t_finish)**2 * direction
        # add the final step, at the target position
        planPos[:,-1] = posOffset
        planVel[:,-1] = 0.0
        return planPos, planVel, numSteps+1, t_finish
    
    def _vertical_body_plan(self, height):
        """Create the plan for vertical movement
        height is current height
        returns planPos, planVel, numSteps where numSteps is the number of timesteps the plan will take,
        planPos and planVel are size numSteps arrays of vertical position and velocity points"""
        #uses constant acceleration model
        direction = np.sign(height - self._hstar)
        t_finish = np.sqrt(direction * 2 * (height - self._hstar) / self._astar)
        numSteps = int(np.ceil(t_finish / self.dt))
        planPos = np.empty((numSteps+1))
        planVel = np.empty((numSteps+1))
        for i in range(numSteps):
            planPos[i] = self._hstar + self._astar / 2 * (self.dt * i - t_finish)**2 * direction
            planVel[i] = self._astar * (self.dt * i - t_finish) * direction
        # add the final step, at the target position
        planPos[-1] = self._hstar
        planVel[-1] = 0.0
        return planPos, planVel, numSteps+1, t_finish
    
    def _leg_plan(self, leg_pos, body_pos, body_vel, num_steps):
        """create a plan for what the leg will do given a body plan.
        leg_pos is current leg position as a 3-vector,
        body_pos is the plan for the body position, body_vel is planned velocity, both 3xN
        returns (pos, vel) both (3xN) for leg FORWARD, OUT, and UP
        """
        planPos = leg_pos.reshape(3,1) * np.ones((1, num_steps))
        planVel = np.zeros((3,num_steps))
        #on_ground = planPos[UP,:] > 2e-3 # True if the leg is supposed to be on the ground supporting weight
        planPos[UP,:] = 0.0
        return planPos, planVel
        
    @property
    def pos(self):
        return self._pos
    @property
    def vel(self):
        return self._vel
    
    @property
    def numSteps(self):
        return self._numSteps
    @property
    def planTime(self):
        return self._tf
    
    @property
    def rightPos(self):
        return self._rightPos
    @property
    def rightVel(self):
        return self._rightVel
    @property
    def leftPos(self):
        return self._leftPos
    @property
    def leftVel(self):
        return self._leftVel


def test():
    numTries = 1000
    makePlot = True
    import time
    import kinematics
    planner = MotionPlanner()
    testPos = np.zeros((3,numTries))
    testPos[UP,:] = planner._hstar + 0.02 * np.random.standard_normal((1,numTries))
    testVel = np.random.standard_normal((3,numTries)) * 0.03
    legPos = np.zeros(3)
    legPos[kinematics.RIGHT] = 0.03
    allNumSteps = np.empty(numTries)
    
    #test timing
    startTime = time.perf_counter()
    for i in range(numTries):
        planner.createPlan(testPos[:,i], legPos, LEFTSCALE*legPos, testVel[:,i])
        allNumSteps[i] = planner.numSteps
    planTime = time.perf_counter() - startTime
    print("Plan creation takes", round(planTime * 1e6 / numTries, 3), "us")
    
    numTries = 100
    for i in range(numTries):
        planner.createPlan(testPos[:,i], legPos, LEFTSCALE*legPos, testVel[:,i])
        #print(planner.numSteps, "steps")
        #check shape consistency
        shape = (3, planner.numSteps)
        if not (planner.pos.shape == shape and planner.vel.shape == shape):
            print("FAIL: pos or vel wrong shape", planner.pos.shape, planner.vel.shape)
        if not (planner.rightPos.shape == shape and planner.leftPos.shape == shape and planner.rightVel.shape == shape and planner.leftVel.shape == shape):
            print("FAIL: leg plan wrong shape", planner.rightPos.shape, planner.leftPos.shape, planner.rightVel.shape, planner.leftVel.shape)
            
        # check that first step of plan is current pos and vel
        if not np.all(np.isclose(planner.pos[:,0], testPos[:,i])):
            print("FAIL: plan from velocity", testVel[:,i], "starts at wrong position", planner.pos[:,0])
        if not np.all(np.isclose(planner.vel[HORIZONTAL,0],
                                 testVel[HORIZONTAL,i])):
            print("FAIL: plan from velocity", testVel[:,i], "starts at wrong velocity", planner.vel[:,0])
        
        # check dt and numSteps match
        if     (planner.numSteps-2) * planner.dt > planner.planTime \
            or (planner.numSteps-1) * planner.dt < planner.planTime:
            print("FAIL: plan time", planner.planTime, "doesn't match numSteps", planner.numSteps, planner.numSteps*planner.dt)
        
        # check one leg is still on ground
        right_on_ground = np.logical_and(np.abs(planner.rightPos[UP,1:]) < 1e-5, 
                                         np.sum(np.abs(np.diff(planner.rightPos[HORIZONTAL,:])),axis=0) < 1e-5)
        left_on_ground = np.logical_and(np.abs(planner.leftPos[UP,1:]) < 1e-5, 
                                         np.sum(np.abs(np.diff(planner.leftPos[HORIZONTAL,:])),axis=0) < 1e-5)
        if not np.all(np.logical_or(right_on_ground, left_on_ground)):
            print("FAIL: neither leg on ground")
            print("Right", np.sum(np.abs(np.diff(planner.rightPos[HORIZONTAL,:])),axis=0),
                    "Left", np.sum(np.abs(np.diff(planner.leftPos[HORIZONTAL,:])),axis=0))
        
        # test advancing
        import copy
        if planner.numSteps > 1:
            plan_cp = copy.deepcopy(planner)
            plan_cp.advance()
            if ((not np.all(np.isclose(plan_cp.pos[:,0], planner.pos[:,1])))
                or (not np.all(np.isclose(plan_cp.vel[:,0], planner.vel[:,1])))
                or (not np.all(np.isclose(plan_cp.rightPos[:,0], planner.rightPos[:,1])))
                or (not np.all(np.isclose(plan_cp.leftPos[:,0], planner.leftPos[:,1])))
                or (not np.all(np.isclose(plan_cp.rightVel[:,0], planner.rightVel[:,1])))
                or (not np.all(np.isclose(plan_cp.leftVel[:,0], planner.leftVel[:,1])))
                or (plan_cp.planTime > 1e-15 and not np.isclose(plan_cp.planTime + plan_cp.dt, planner.planTime))
                ):
                print("Didn't advance correctly")
                
        
        # last, check that replanning starting at predicted step k+i doesn't change plans for k+j >= k+i
        if planner.numSteps > 3:
            checkIndex = np.random.randint(1, planner.numSteps-2)
            plannedPos = planner.pos[:,checkIndex+1]
            plannedVel = planner.vel[:,checkIndex+1]
            planner.createPlan(planner.pos[:,checkIndex], legPos, LEFTSCALE*legPos, planner.vel[:,checkIndex])
            if not (np.all(np.isclose(plannedPos, planner.pos[:,1]))
                and np.all(np.isclose(plannedVel, planner.vel[:,1]))):
                print("FAIL: replanning velocity", testVel[:,i], "changed", checkIndex, planner.pos[:,0], planner.vel[:,0],
                      "from\n", plannedPos, "to", planner.pos[:,1], plannedVel, "to", planner.vel[:,1])
    if makePlot:
        try:
            import matplotlib.pyplot as plt
        except:
            pass
        else:
            fig = plt.figure(1)
            fig.clf()
            plt.hist(allNumSteps)
            plt.show()

if __name__ == "__main__":
    test()
    