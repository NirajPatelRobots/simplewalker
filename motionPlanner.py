# -*- coding: utf-8 -*-
"""
Motion planning for walking robot
TODO:
    other behaviors like sway back + forth and walk
    kick off for stepping
    get into the various t_f and array length issues. Clarify and maybe speed up with less allocation?
    smoother step

Created Mar 2021
@author: Niraj
"""
import numpy as np
from kinematics import UP, FORWARD, RIGHT, HORIZONTAL, UP_DIR, HORIZONTAL_DIR

class MotionPlanner:
    """Class for a motion planner.
    Use MotionPlanner.createPlan to create a plan.
    Use MotionPlanner.leg_plan to calculate leg behaviour once a plan is created.
    Once a plan is created, MotionPlanner.pos is the 3xnumSteps array of positions,
    MotionPlanner.vel is the 3xnumSteps array of velocities.
    MP.rightPos, rightVel, leftPos, and leftVel are the 3x2 leg positions and velocities in robot coordinates
    The first elements are (3x1) current leg status, and the second elements are (3x1) planned leg status at next timestep
    rightPos and leftPos are NOT relative to connection point
    MotionPlanner.numSteps is the number of points in the plan.
    MotionPlanner.planTime is the length of the plan in seconds."""
    def __init__(self, horizJerk=0.1, heightAccel=2.0, targetHeight=0.15, dt=1/40, step_speed = 1., step_height = 0.02):
        """horizJerk is the constant Jerk for the horizontal movement.
        heightAccel is constant acceleration upwards or downwards towards target height
        targetHeight [m] is the desired body height
        dt [s] is the time between points in the plan
        step_speed is how fast the leg moves when stepping, roughly 1/"reaction time"
        step_height is a maximum in meters [m]. The foot might not get that high, esp. for short steps.
        """
        self._max_steps = 5 # maximum number of timesteps forward the prediction uses
        self._pos = np.empty((3,self._max_steps+1))
        self._vel = np.empty((3,self._max_steps+1))
        self._leftPos = np.empty((3,2))
        self._rightPos = np.empty((3,2))
        self._leftVel = np.empty((3,2))
        self._rightVel = np.empty((3,2))
        self._numSteps = 0
        self._tf = 0.0
        self._Jstar = horizJerk
        self._astar = heightAccel
        self._hstar = targetHeight
        self._foot_width = 0.04 # [m]
        self._lift_foot_scale = 0.5
        self.dt = dt
        self.step_speed = step_speed
        self.step_height = step_height
        start_pos = np.zeros(3)
        start_pos[UP] = self._hstar
        self.createPlan(start_pos, np.zeros(3))

    def createPlan(self, curr_pos, curr_vel):
        """ creates a plan based on current position and velocity.
        curr_pos, curr_vel are 3-arrays of floats for leg position
        Does not set the leg plan"""
        pos, vel, h_steps, h_time = self._push_around_plan(curr_pos, curr_vel)
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
    
    def leg_plan(self, right_pos, right_vel, left_pos, left_vel):
        """create a plan for what the leg will do given a body plan.
        Uses the existing plan for the body created by createPlan.
        left_pos and right_pos are current leg positions in robot coordinates as a 3-vector,
        sets leg pos and vel both (3x2) for leg FORWARD, RIGHT, and UP, this timestep and next."""
        self._rightPos[:,0] = right_pos
        self._leftPos[:,0] = left_pos
        self._rightVel[:,0] = right_vel
        self._leftVel[:,0] = left_vel
        #decide which leg is swing leg and which is still leg
        if (right_pos[UP] > 0.001 or
            (left_pos[UP] < 0.001 and np.linalg.norm(self._rightPos[HORIZONTAL,0]) > np.linalg.norm(self._leftPos[HORIZONTAL,0]))):
            leg_acc = self._plan_step(self._pos[:,0], self._pos[:,self.numSteps-1], right_pos, right_vel, left_pos)
            self._rightVel[:,1] = self._rightVel[:,0] + leg_acc * self.dt
            self._leftVel[:,1] = np.zeros(3)
        else: #left leg is swing leg
            leg_acc = self._plan_step(self._pos[:,0], self._pos[:,self.numSteps-1], left_pos, left_vel, right_pos)
            self._leftVel[:,1] = self._leftVel[:,0] + leg_acc * self.dt
            self._rightVel[:,1] = np.zeros(3)
        self._rightPos[:,1] = self._rightPos[:,0] + (self._rightVel[:,1] - self._vel[:,0] * HORIZONTAL_DIR) * self.dt
        self._leftPos[:,1] = self._leftPos[:,0] + (self._leftVel[:,1] - self._vel[:,0] * HORIZONTAL_DIR) * self.dt
        #using updated velocity to update the position is a *spicy* physical programming technique, gotta jazz it up a bit

    def advance(self):
        """remove the current timestep, advancing so that the former second step is now the first step.
        Will not remove the last point in a plan."""
        if self._numSteps > 1:
            self._pos = self._pos[:,1:]
            self._vel = self._vel[:,1:]
            self._numSteps -= 1
            self._tf = max(self._tf - self.dt, 0.0)
        else:
            self._tf = max(self._tf - self.dt, 0.0)
        
        
    def _push_around_plan(self, pos, vel):
        """Create the horizontal part of a plan.
        pos and vel are both 3-arrays of floats.
        returns planPos, planVel, numSteps where numSteps is the number of timesteps the plan will take,
        planPos and planVel are 3xnumSteps arrays of position and velocity points.
        planPos[UP] and planVel[UP] are undefined because this method only plans the horizontal part."""
        #uses constant Jerk model
        vel = np.copy(vel)
        vel[UP] = 0.
        speed = np.linalg.norm(vel)
        if speed < 1e-12:
            direction = np.zeros(3)
        else:
            direction = vel / speed
        t_finish = np.sqrt(2 * speed / self._Jstar) # time to finish horizontal plan
        numSteps = min(int(np.ceil(t_finish / self.dt)), self._max_steps)
        posOffset = pos + self._Jstar / 6 * t_finish**3 * direction
        planPos = np.empty((3,numSteps))
        planVel = np.empty((3,numSteps))
        for i in range(numSteps):
            planPos[:,i] = posOffset + self._Jstar / 6 * (self.dt * i - t_finish)**3 * direction
            planVel[:,i] = self._Jstar / 2 * (self.dt * i - t_finish)**2 * direction
        if numSteps < self._max_steps:
            # add the final step, at the target position
            #TODO: something better than this
            planPos = np.hstack((planPos, posOffset.reshape(3,1)))
            planVel = np.hstack((planVel, np.zeros((3,1))))
            return planPos, planVel, numSteps+1, t_finish
        return planPos, planVel, numSteps, self.dt * (self._max_steps - 1)
    
    def _vertical_body_plan(self, height):
        """Create the plan for vertical movement
        height is current height
        returns planPos, planVel, numSteps where numSteps is the number of timesteps the plan will take,
        planPos and planVel are size numSteps arrays of vertical position and velocity points"""
        #uses constant acceleration model
        direction = np.sign(height - self._hstar)
        t_finish = np.sqrt(direction * 2 * (height - self._hstar) / self._astar)
        numSteps = min(int(np.ceil(t_finish / self.dt)), self._max_steps)
        planPos = np.empty((numSteps))
        planVel = np.empty((numSteps))
        for i in range(numSteps):
            planPos[i] = self._hstar + self._astar / 2 * (self.dt * i - t_finish)**2 * direction
            planVel[i] = self._astar * (self.dt * i - t_finish) * direction
        if numSteps < self._max_steps:
            # add the final step, at the target position
            #TODO: is this really a good idea
            planPos = np.concatenate((planPos, [self._hstar]))
            planVel = np.concatenate((planVel, [0.0]))
            return planPos, planVel, numSteps+1, t_finish
        return planPos, planVel, numSteps, (self._max_steps - 1) * self.dt            

    
    def _plan_step(self, body_pos, target_body_pos, leg_pos, leg_vel, alt_leg_pos):
        """create a single step where a leg lifts and is set down.
        body_pos is current body position
        target_body_pos is body position in future, where we're analyzing the step
        leg_pos and leg_vel are current swing leg position
        alt_leg_pos is current standing leg position
        returns intended swing leg acceleration as a 3-array
        """
        #analyze stability of target_body pos with future leg pos
        future_leg_pos = leg_pos + leg_vel * self._tf
        future_alt_leg_pos = alt_leg_pos + (body_pos - target_body_pos)
        target_leg_pos = np.empty(3)
        if self._is_stable(future_leg_pos, future_alt_leg_pos):
            target_leg_pos[UP] = 0.0
            target_leg_pos[RIGHT] = leg_pos[RIGHT]
            target_leg_pos[FORWARD] = leg_pos[FORWARD]
        #elif leg_pos[UP] > 0.001 and self._is_stable(leg_pos, alt_leg_pos): #on ground and currently stable
            #TODO kick off ground
        else: #unstable 
            #lift foot, move to better position
            target_leg_pos[UP] = self.step_height
            target_leg_pos[RIGHT] = -alt_leg_pos[RIGHT]
            target_leg_pos[FORWARD] = -alt_leg_pos[FORWARD]
        leg_acc = self._kstep * (target_leg_pos - future_leg_pos)
        leg_acc[UP] *= self._lift_foot_scale
        return leg_acc
        
    def _is_stable(self, leg_pos, alt_leg_pos):
        # consult the Stability Rhombus 
        leg_dist = ((alt_leg_pos[FORWARD] - leg_pos[FORWARD]) / (alt_leg_pos[RIGHT] - leg_pos[RIGHT] + np.sign(alt_leg_pos[RIGHT]) * self._foot_width)
                    * -leg_pos[RIGHT]) + leg_pos[FORWARD]
        alt_leg_dist = ((leg_pos[FORWARD] - alt_leg_pos[FORWARD]) / (leg_pos[RIGHT] - alt_leg_pos[RIGHT] + np.sign(leg_pos[RIGHT]) * self._foot_width)
                    * -alt_leg_pos[RIGHT]) + alt_leg_pos[FORWARD]
        return (leg_dist > 0.) ^ (alt_leg_dist > 0.) # do leg_dist and alt_leg_dist have the opposite sign
            
        
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
    def step_speed(self):
        return np.sqrt(self._kstep / 8 / np.pi)
    @step_speed.setter
    def step_speed(self, speed):
        self._kstep = (speed * 8 * np.pi)**2
    
    
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
    legVel = np.zeros(3)
    legPos[kinematics.RIGHT] = 0.03
    legPos[kinematics.FORWARD] = 0.03
    allNumSteps = np.empty(numTries)
    
    #test timing
    startTime = time.perf_counter()
    for i in range(numTries):
        planner.createPlan(testPos[:,i], testVel[:,i])
        allNumSteps[i] = planner.numSteps
    planTime = time.perf_counter() - startTime
    print("Plan creation takes", round(planTime * 1e6 / numTries, 3), "us")
    
    startTime = time.perf_counter()
    for i in range(numTries):
        planner.createPlan(testPos[:,i], testVel[:,i])
        planner.leg_plan(legPos, legVel, -legPos, legVel)
        allNumSteps[i] = planner.numSteps
    planTime = time.perf_counter() - startTime
    print("Plan creation with legs takes", round(planTime * 1e6 / numTries, 3), "us")
    
    numTries = 100
    for i in range(numTries):
        planner.createPlan(testPos[:,i], testVel[:,i])
        planner.leg_plan(legPos, legVel, -legPos, legVel)
        #print(planner.numSteps, "steps")
        #check shape consistency
        shape = (3, planner.numSteps)
        if not (planner.pos.shape == shape and planner.vel.shape == shape):
            print("FAIL: pos or vel wrong shape", planner.pos.shape, planner.vel.shape)
            
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
        right_on_ground = np.abs(planner.rightPos[UP,1:]) < 1e-5
        left_on_ground = np.abs(planner.leftPos[UP,1:]) < 1e-5
        #TODO: test grounded leg stays still
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
                or (plan_cp.planTime > 1e-15 and not np.isclose(plan_cp.planTime + plan_cp.dt, planner.planTime))
                ):
                print("Didn't advance correctly")
        
        # check that replanning starting at predicted step k+i doesn't change plans for k+j >= k+i
        if planner.numSteps > 3:
            checkIndex = np.random.randint(1, planner.numSteps-2)
            plannedPos = planner.pos[:,checkIndex+1]
            plannedVel = planner.vel[:,checkIndex+1]
            planner.createPlan(planner.pos[:,checkIndex], planner.vel[:,checkIndex])
            if not (np.all(np.isclose(plannedPos, planner.pos[:,1]))
                and np.all(np.isclose(plannedVel, planner.vel[:,1]))):
                print("FAIL: replanning velocity", testVel[:,i], "changed", checkIndex, planner.pos[:,0], planner.vel[:,0],
                      "from\n", plannedPos, "to", planner.pos[:,1], plannedVel, "to", planner.vel[:,1])
    print("Test step")
    # give an initial velocity that would cause a step
    stepVel = np.zeros(3)
    stepVel[RIGHT] = -0.035
    stepVel[FORWARD] = 0.06
    leg_up = False
    i = 0
    planner.createPlan(planner._hstar * UP_DIR, stepVel)
    planner.leg_plan(legPos, legVel, -1.2 * legPos, legVel)
    bodylog = (planner._hstar * UP_DIR).reshape(3,1)
    swinglog = planner.leftPos[:,0].reshape(3,1)
    stilllog = planner.rightPos[:,0].reshape(3,1)
    print("body position", planner.pos[:,0], "right foot position", planner.rightPos[:,0],
          "left foot position", planner.leftPos[:,0])
    # run until done
    while planner.numSteps > 1 or (planner.leftPos[UP,0] > 0.001 and i < 5 / planner.dt):
        if planner.numSteps > 2:
            planner.createPlan(planner.pos[:,1], planner.vel[:,1])
        else:
            print(planner.numSteps)
            pass
        planner.leg_plan(planner.rightPos[:,1], planner.rightVel[:,1], planner.leftPos[:,1], planner.leftVel[:,1])
        bodylog = np.hstack((bodylog, planner.pos[:,0:1]))
        swinglog = np.hstack((swinglog, planner.leftPos[:,0:1]))
        stilllog = np.hstack((stilllog, planner.rightPos[:,0:1]))
        if not leg_up:
            if planner.leftPos[UP,0] > 0.0001:
                print("lift leg at", i)
                leg_up = True
        else:
            if planner.leftPos[UP,0] < 0.0001:
                print("leg down at", i)
                leg_up = False
        if planner.rightPos[UP,0] > 0.0001:
            print("Right leg incorrectly lifted")
            break
        i += 1
    # check end is stable
    if planner._is_stable(planner.rightPos[:,0], planner.leftPos[:,0]):
        print("Ends stable after", i * planner.dt, "[s] at:")
    else:
        print("Step ends unstable (BAD!!) after", i * planner.dt, "[s] at")
    print("body position", planner.pos[:,0], "right foot position", planner.rightPos[:,0],
           "left foot position", planner.leftPos[:,0])
    print("body velocity", planner.vel[:,0], "right foot velocity", planner.rightVel[:,0],
      "left foot velocity", planner.leftVel[:,0])
    if makePlot:
        try:
            import matplotlib.pyplot as plt
        except:
            pass
        else:
            n = np.shape(bodylog)[1]
            fig = plt.figure(1)
            fig.clf()
            ax = fig.add_subplot(projection = '3d')
            #plt.subplot(2, 1, 1)
            ax.scatter(stilllog[RIGHT,:], stilllog[FORWARD,:], zs = stilllog[UP,:], c = np.arange(n), cmap = "viridis")
            ax.scatter(swinglog[RIGHT,:], swinglog[FORWARD,:], zs = swinglog[UP,:], c = np.arange(n), cmap = "plasma")
            plt.grid(True)
            plt.xlabel("Sideways (+ right) [m]")
            plt.ylabel("Forward [m]")
            ax.set_zlabel("Height [m]")
            plt.title("Leg positions")
            fig = plt.figure(2)
            fig.clf()
            ax = fig.add_subplot(projection = '3d')
            #plt.subplot(2, 1, 1)
            ax.scatter(bodylog[RIGHT,:], bodylog[FORWARD,:], c = np.arange(n), cmap = "cividis")
            ax.scatter(bodylog[RIGHT,:] + stilllog[RIGHT,:], bodylog[FORWARD,:] + stilllog[FORWARD,:], zs = stilllog[UP,:], c = np.arange(n), cmap = "viridis")
            ax.scatter(bodylog[RIGHT,:] + swinglog[RIGHT,:], bodylog[FORWARD,:] + swinglog[FORWARD,:], zs = swinglog[UP,:], c = np.arange(n), cmap = "plasma")
            plt.grid(True)
            plt.xlabel("Sideways (+ right) [m]")
            plt.ylabel("Forward [m]")
            ax.set_zlabel("Height [m]")
            plt.title("Global positions")
            

if __name__ == "__main__":
    test()
    