import math
import time
from rlbot.agents.base_agent import  SimpleControllerState
from Util import *

class seven:
    def __init__(self):
        self.expired= False
        self.jumping = False
        self.time = time.time()
    def execute(self,agent):
        if self.jumping == False and agent.me.grounded:
            self.jumping = True
            test = dpp(agent.ball.location,agent.ball.velocity,agent.me.location,agent.me.velocity)/1.5
            eta = math.sqrt(((agent.ball.location - agent.me.location).magnitude()+test)/525)
            self.time = time.time() + eta
            target = Vector3([0,0,0])
        else:
            time_remain = cap(self.time - time.time(),-2,10)
            if time_remain != 0:
                target = future(agent.ball, time_remain)
            else:
                time_remain = 0.1
                target = future(agent.ball, 0.1)
            oops = Vector3(target.data)
        
            if time_remain > -1.9:
                target = backsolveFuture(agent.me.location,agent.me.velocity, target,time_remain)
                agent.renderer.begin_rendering()
                agent.renderer.draw_line_3d(agent.me.location.data, targetFuture(agent.me.location,agent.me.velocity,time_remain).data, agent.renderer.create_color(255,255,190,0))
                agent.renderer.draw_rect_3d(targetFuture(agent.me.location,agent.me.velocity,time_remain).data, 10,10, True, agent.renderer.create_color(255,255,190,0))
                agent.renderer.draw_line_3d(agent.me.location.data, oops.data, agent.renderer.create_color(255,0,190,255))
                agent.renderer.draw_rect_3d(oops.data, 10,10, True, agent.renderer.create_color(255,0,190,255))
                agent.renderer.end_rendering()
            else:
                self.expired = True
                target = agent.me.velocity
                self.jumping = False
        return deltaC(agent,target)

def deltaC(agent, target):
    c = SimpleControllerState()
    target = target# - agent.me.velocity
    target_local = toLocal(agent.me.location + target,agent.me)
    angle_to_target = math.atan2(target_local.data[1],target_local.data[0])
    pitch_to_target = math.atan2(target_local.data[2],target_local.data[0])  
    
    if agent.me.grounded:
        if agent.jt + 1.5 > time.time():
            c.jump = True
        else:
            c.jump = False
            agent.jt = time.time()
    else:
        c.yaw = steerPD(angle_to_target,-agent.me.rvelocity.data[1]/4)
        c.pitch = steerPD(pitch_to_target,agent.me.rvelocity.data[0]/4)
        if target.magnitude() > 10:
            c.boost = True
        if abs(pitch_to_target) + abs(angle_to_target) > 0.9:
            c.boost = False
        else:
            top = toLocal([agent.me.location.data[0],agent.me.location.data[1],agent.me.location.data[2]+1000],agent.me)
            roll_to_target = math.atan2(top.data[1],top.data[2])
            c.roll = steerPD(roll_to_target,agent.me.rvelocity.data[2]*0.5)
        tsj = time.time() - agent.jt
        if tsj < 0.2:
            c.jump = True
        elif tsj < 0.25:
            c.jump = False
        elif tsj >=0.25 and tsj < 0.27 and target.data[2]>350:
            c.jump = True
            c.boost = False
            c.yaw = 0
            c.pitch = 0
            c.roll = 0
        else:
            c.jump = False            
    return c


def sevenC(agent,target,speed):
    c = SimpleControllerState()

    #target.data[2] = target.data[2]-90#futureZ(target.data[2],agent.ball.velocity.data[2], 0.2)
    
    angle = angle2(target,agent.me.location)
    x_speed = (math.cos(angle) * speed*1.1) - (agent.me.velocity.data[0])
    y_speed = (math.sin(angle) * speed*1.1) - (agent.me.velocity.data[1])
    #print(x_speed,y_speed)
    
    correction_vector = Vector3([x_speed/2300,y_speed/2300,speed/2300])

    target_local = toLocal(agent.me.location + correction_vector, agent.me)
    angle_to_target = math.atan2(target_local.data[1],target_local.data[0])
    pitch_to_target = math.atan2(target_local.data[2],target_local.data[0])  
    
    if agent.me.grounded:
        if agent.jt + 1.5 > time.time():
            c.jump = True        
        else:
            c.jump = False
            agent.jt = time.time()
    else:
        if agent.jt > time.time() - 1.5:
            c.jump = True
        else:
            c.jump = False
        c.throttle,c.boost = altPD(target.data[2]-agent.me.location.data[2],agent.me.velocity.data[2])
        c.yaw = steerPD(angle_to_target,-agent.me.rvelocity.data[1]/5)
        c.pitch = steerPD(pitch_to_target,agent.me.rvelocity.data[0]/4)
        if abs(pitch_to_target) + abs(angle_to_target) < 0.5:
            two = target
            loctwo = toLocal(two,agent.me)
            to_two = math.atan2(loctwo.data[1],loctwo.data[2])
            c.roll = steerPD(to_two,agent.me.rvelocity.data[2]/3)
        if abs(pitch_to_target) + abs(angle_to_target) > 0.45:
            c.boost = False

    return c

class six():
    def __init__(self):
        self.expired = False
    def execute(self,agent):
        adjust = agent.me.location + (agent.ball.location - agent.me.location).normalize().div(.0009)
        #spacing = 150
        #index = [agent.index]
        '''
        team = 1
        x = 1
        for car in agent.players:
            if car.team == agent.team:
                adjust += car.location
                x+=1
                team += 1
                index.append(car.index)
            else:
                adjust += car.location.div(0.25)
                x+=4                          
        adjust = adjust.div(x)
        if team == 4:
            index.sort()
            if index.index(agent.index) == 0:
                adjust += Vector3([spacing,spacing,0])
            if index.index(agent.index) == 1:
                adjust += Vector3([spacing,-spacing,0])
            if index.index(agent.index) == 2:
                adjust += Vector3([-spacing,spacing,0])
            if index.index(agent.index) == 3:
                adjust += Vector3([-spacing,-spacing,0])
        '''
        '''
        leftPost = Vector3([-sign(agent.team)*700 , 5100*sign(agent.team), 500])
        for car in agent.players:
            if car.team == agent.team:
                index.append(car.index)
        index.sort()
        adjust += Vector3([spacing*index.index(agent.index),0,0])
        '''
        v = agent.me.velocity.data
        target = Vector3([adjust.data[0],adjust.data[1],adjust.data[2]])
        opposite = Vector3([-v[0],-v[1],0])#Vector3([agent.me.location.data[0],agent.me.location.data[1],1000])
        target += opposite.div(1)#agent.me.velocity.magnitude())  
        
        speed = 2300
        return sixC(agent,target,speed)

    
def sixC(agent,target,speed):
    controller_state = SimpleControllerState()
    target_relative = target.location - agent.me.location
    target_vel_relative = target.velocity - agent.me.velocity
    target_local = toLocal(target,agent.me)
    agent_local = toLocal(agent.me.velocity,agent.me)
    angle_to_target = math.atan2(target_local.data[1],target_local.data[0])
    now = time.time()
    if agent.me.grounded:
        agent_speed = velocity2D(agent.me)
        controller_state.steer = steer(angle_to_target)
        controller_state.throttle, controller_state.boost = throttle(speed, agent_speed)
        if target_local.data[2] > target_local.data[0]*3 and now > agent.jt + 1:
            controller_state.jump = True
            agent.jt = now
        return controller_state
    else:       
        agent_speed = velocity2D(agent.me)
        pitch_to_target = math.atan2(target_local.data[2],target_local.data[0])
        controller_state.jump = False
        controller_state.yaw = steerPD(angle_to_target,-agent.me.rvelocity.data[1]/5)
        controller_state.pitch = steerPD(pitch_to_target,agent.me.rvelocity.data[0]/4)
        controller_state.throttle, controller_state.boost = altPD(target_relative.data[2],agent.me.velocity.data[2])
        if abs(pitch_to_target) > 0.4:
            controller_state.boost = False
        if abs(pitch_to_target) + abs(angle_to_target) < 0.5:
            two = agent.ball.location
            loctwo = toLocal(two,agent.me)
            to_two = math.atan2(loctwo.data[1],loctwo.data[2])
            controller_state.roll = steerPD(to_two,agent.me.rvelocity.data[2]/4)
            
        return controller_state   
