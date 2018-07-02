import math
import time
from rlbot.agents.base_agent import  SimpleControllerState
from Util import *

class exampleATBA:
    def __init__(self):
        self.expired = False

    def execute(self, agent):
        target_location = agent.ball
        target_speed = velocity2D(agent.ball) + (distance2D(agent.ball,agent.me)/1.5)
        
        return agent.controller(agent, target_location, target_speed)
    
class quickShot:
    def __init__(self):
        self.expired = False
    def execute(self,agent):
        agent.controller = shotController
        left_post = Vector3([sign(agent.team)*GOAL_WIDTH/2,-sign(agent.team)*FIELD_LENGTH/2,100])
        right_post = Vector3([-sign(agent.team)*GOAL_WIDTH/2,-sign(agent.team)*FIELD_LENGTH/2,100])
        
        ball_left = angle2(agent.ball.location,left_post)
        ball_right = angle2(agent.ball.location,right_post)

        our_left = angle2(agent.me.location,left_post)
        our_right = angle2( agent.me.location,right_post)

        target_speed = 1399

        if our_left <= ball_left and our_right >= ball_right:
            target_location = toLocation(agent.ball)
        elif our_left > ball_left and our_right >= ball_right: #ball is too far right
            target_location = toLocation([agent.ball.location.data[0],agent.ball.location.data[1]+sign(agent.team)*160,agent.ball.location.data[2]])
        elif our_right < ball_right and our_left <= ball_left: #ball is too far left
            target_location = toLocation([agent.ball.location.data[0],agent.ball.location.data[1]+sign(agent.team)*160,agent.ball.location.data[2]])
        else:
            target_location = toLocation([0,sign(agent.team)*FIELD_LENGTH/2,100])

        return agent.controller(agent,target_location, target_speed)

    
def exampleController(agent, target_object,target_speed):
    location = toLocal(target_object,agent.me)
    controller_state = SimpleControllerState()
    angle_to_ball = math.atan2(location.data[1],location.data[0])

    current_speed = velocity2D(agent.me)
    #steering
    if angle_to_ball > 0.1:
        controller_state.steer = controller_state.yaw = 1
    elif angle_to_ball < -0.1:
        controller_state.steer = controller_state.yaw = -1
    else:
        controller_state.steer = controller_state.yaw = 0

    #throttle
    if target_speed > current_speed:
        controller_state.throttle = 1.0
        if target_speed > 1400 and agent.start > 2.2 and current_speed < 2250:
            controller_state.boost = True
    elif target_speed < current_speed:
        controller_state.throttle = 0

    #dodging
    time_difference = time.time() - agent.start
    if time_difference > 2.2 and distance2D(target_object,agent.me) > 1000 and abs(angle_to_ball) < 1.3:
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

def shotController(agent, target_object,target_speed):
    goal_local = toLocal([0,-sign(agent.team)*FIELD_LENGTH/2,100],agent.me)
    goal_angle = math.atan2(goal_local.data[1],goal_local.data[0])

    location = toLocal(target_object,agent.me)
    controller_state = SimpleControllerState()
    angle_to_target = math.atan2(location.data[1],location.data[0])

    current_speed = velocity2D(agent.me)
    #steering
    if angle_to_target > 0.1:
        controller_state.steer = controller_state.yaw = 1
    elif angle_to_target < -0.1:
        controller_state.steer = controller_state.yaw = -1
    else:
        controller_state.steer = controller_state.yaw = 0

    #throttle
    if angle_to_target >=1.4:
        target_speed -=1400
    else:
        if target_speed > 1400 and target_speed > current_speed and agent.start > 2.2 and current_speed < 2250:
            controller_state.boost = True
    if target_speed > current_speed:
        controller_state.throttle = 1.0
    elif target_speed < current_speed:
        controller_state.throttle = 0

    #dodging
    time_difference = time.time() - agent.start
    if time_difference > 2.2 and distance2D(target_object,agent.me) <= 270:
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
