import math
import time
from rlbot.agents.base_agent import  SimpleControllerState
from Util import *


class calcShot:
    def __init__(self):
        self.expired = False

    def available(self,agent):
        if ballReady(agent) and ballProject(agent) > 500 and abs(agent.ball.location.data[0]) < 3900:
            return True
        return False

    def execute(self,agent):
        agent.controller = calcController
        goal = Vector3([0,-sign(agent.team)*FIELD_LENGTH/2,100])
        goal_local = toLocal(goal,agent.me)
        goal_to_ball = (agent.ball.location - goal).normalize()
        
        goal_to_agent = (agent.me.location - goal).normalize()
        difference = goal_to_ball - goal_to_agent
        error = cap(abs(difference.data[0]) + abs(difference.data[1]),1,10)
        
        target_distance = (100 +distance2D(agent.ball,agent.me) * (error**2) )/ 1.95
        target_location = agent.ball.location + Vector3([goal_to_ball.data[0]*target_distance,goal_to_ball.data[1]*target_distance, 0])
        target_location.data[0] = cap(target_location.data[0],-4120,4120)
        
        target_local = toLocal(target_location,agent.me)
        angle_to_target = math.atan2(target_local.data[1], target_local.data[0])
        distance_to_target = distance2D(agent.me, target_location)
        speedCorrection =  ((1+ abs(angle_to_target)**2) * 300)
        speed = 2300 - speedCorrection + cap((distance_to_target/16)**2,0,speedCorrection)

        if ballProject(agent) < 10:
            self.expired = True

        return agent.controller(agent,target_location,speed)
        

class quickShot:
    def __init__(self):
        self.expired = False

    def available(self,agent):
        if ballReady(agent) and ballProject(agent) > -500:
            return True
        return False

    def execute(self,agent):
        agent.controller = shotController
        left_post = Vector3([sign(agent.team)*GOAL_WIDTH/2,-sign(agent.team)*FIELD_LENGTH/2,100])
        right_post = Vector3([-sign(agent.team)*GOAL_WIDTH/2,-sign(agent.team)*FIELD_LENGTH/2,100])
        
        ball_left = angle2(agent.ball.location,left_post)
        ball_right = angle2(agent.ball.location,right_post)

        our_left = angle2(agent.me.location,left_post)
        our_right = angle2( agent.me.location,right_post)

        offset = (agent.ball.location.data[0] / FIELD_WIDTH) * 3.14
        x = agent.ball.location.data[0] +100 * abs(math.cos(offset)) * sign(offset)
        y = agent.ball.location.data[1] + 100 * abs(math.sin(offset)) * sign(agent.team)
        target_location = toLocation([x,y,agent.ball.location.data[2]])

        location = toLocal(target_location,agent.me)
        angle_to_target = math.atan2(location.data[1],location.data[0])
        distance_to_target = distance2D(agent.me, target_location)
        
        speedCorrection =  ((1+ abs(angle_to_target)**2) * 300)
        speed = 2000 - speedCorrection + cap((distance_to_target/16)**2,0,speedCorrection)
        
        
        
        if distance2D(agent.me.location,agent.ball.location) < 400 and abs(angle_to_target) > 2:
            self.expired = True
        elif calcShot().available(agent) == True:
            self.expired = True

        return agent.controller(agent,target_location, speed)

def calcController(agent, target_object,target_speed):
    location = toLocal(target_object,agent.me)
    controller_state = SimpleControllerState()
    angle_to_ball = math.atan2(location.data[1],location.data[0])

    current_speed = velocity2D(agent.me)
    controller_state.steer = steer(angle_to_ball)

    #throttle
    if target_speed > current_speed:
        controller_state.throttle = 1.0
        if target_speed > 1400 and agent.start > 2.2 and current_speed < 2250:
            controller_state.boost = True
    elif target_speed < current_speed:
        controller_state.throttle = -1.0
    return controller_state

def shotController(agent, target_object,target_speed):
    goal_local = toLocal([0,-sign(agent.team)*FIELD_LENGTH/2,100],agent.me)
    goal_angle = math.atan2(goal_local.data[1],goal_local.data[0])

    location = toLocal(target_object,agent.me)
    controller_state = SimpleControllerState()
    angle_to_target = math.atan2(location.data[1],location.data[0])

    current_speed = velocity2D(agent.me)
    #steering
    controller_state.steer = steer(angle_to_target)

    #throttle
    if target_speed > 1400 and target_speed > current_speed and agent.start > 2.5 and current_speed < 2250:
        controller_state.boost = True
    if target_speed > current_speed:
        controller_state.throttle = 1.0
    elif target_speed < current_speed:
        controller_state.throttle = 0

    #dodging
    time_difference = time.time() - agent.start
    if ballReady(agent) and time_difference > 2.2 and distance2D(target_object,agent.me) <= 270:
        agent.start = time.time()
    elif time_difference <= 0.1:
        controller_state.jump = True
        controller_state.pitch = -1
    elif time_difference >= 0.1 and time_difference <= 0.15:
        controller_state.jump = False
        controller_state.pitch = -1
    elif time_difference > 0.15 and time_difference < 1:
        controller_state.jump = True
        controller_state.yaw = math.sin(goal_angle)
        controller_state.pitch = -abs(math.cos(goal_angle))      

    return controller_state

class exampleATBA:
    def __init__(self):
        self.expired = False
    def execute(self, agent):
        target_location = agent.ball
        target_speed = velocity2D(agent.ball) + (distance2D(agent.ball,agent.me)/1.5)
        
        return agent.controller(agent, target_location, target_speed)

def exampleController(agent, target_object,target_speed):
    location = toLocal(target_object,agent.me)
    controller_state = SimpleControllerState()
    angle_to_ball = math.atan2(location.data[1],location.data[0])

    current_speed = velocity2D(agent.me)
    #steering
    controller_state.steer = steer(angle_to_ball)

    #throttle
    if target_speed > current_speed:
        controller_state.throttle = 1.0
        if target_speed > 1400 and agent.start > 2.2 and current_speed < 2250:
            controller_state.boost = True
    elif target_speed < current_speed:
        controller_state.throttle = 0

    #dodging
    time_difference = time.time() - agent.start
    if time_difference > 2.2 and distance2D(target_object,agent.me) > (velocity2D(agent.me)*2.5) and abs(angle_to_ball) < 1.3:
        agent.start = time.time()
    elif time_difference <= 0.1:
        controller_state.jump = True
        controller_state.pitch = -1
    elif time_difference >= 0.1 and time_difference <= 0.15:
        controller_state.jump = False
        controller_state.pitch = -1
    elif time_difference > 0.15 and time_difference < 1:
        controller_state.jump = True
        controller_state.yaw = controller_state.steer
        controller_state.pitch = -1

    return controller_state
