import math
import time
from Util import *
from States import *
from Experimental import *
import random

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3 as vector3, Rotator

'''
Gosling by ddthj
Episode 5 Code

*now with comments!*
'''

'''
Goal Training
print("reset")
car_state = CarState(physics=Physics(location=vector3(0,2010,20),velocity = vector3(0,0,0),rotation = vector3(0,1.57,0)))
ball_state = BallState(Physics(location=vector3(rd(100),1000,100),velocity=vector3(rd(350),rd(400)+1200,rd(100)+1200),angular_velocity=vector3(0,0,0)))
game_state = GameState(ball = ball_state,cars={self.index: car_state})
self.set_game_state(game_state)
self.jt = time.time() - 2
self.state = seven()
'''

def rd(x):
    return random.randint(-x,x)

class Gosling(BaseAgent):

    def initialize_agent(self):
        self.me = obj()
        self.ball = obj()
        self.players = [] #holds other players in match
        self.start = time.time()

        self.state = circleDrive()
        self.controller = calcController
        self.jt = time.time()

    def checkState(self):
        #print (self.players[0].velocity.data[2])
        if self.state.expired:
            print("reset")
            car_state = CarState(physics=Physics(location=vector3(rd(3500),rd(4500),20),velocity = vector3(0,0,0),rotation = vector3(0,1.57,0)))
            ball_state = BallState(Physics(location=vector3(rd(3500),rd(4500),100),velocity=vector3(rd(350),rd(400),0),angular_velocity=vector3(0,0,0)))
            game_state = GameState(ball = ball_state,cars={self.index: car_state})
            self.set_game_state(game_state)
            self.state = circleDrive()
          
            '''
            if calcShot().available(self) == True and self.ball.location.compare(Vector3([0,0,0])) > 1:
                self.state = calcShot()
            elif quickShot().available(self) == True:
                self.state = quickShot()
            elif wait().available(self) == True:
                self.state = wait()
            else:
                self.state = wait()
        '''
    

    def get_output(self, game: GameTickPacket) -> SimpleControllerState:
        self.preprocess(game)
        self.checkState()
        return self.state.execute(self)

    def preprocess(self,game):
        self.players = []
        car = game.game_cars[self.index]
        self.me.location.data = [car.physics.location.x, car.physics.location.y, car.physics.location.z]
        self.me.velocity.data = [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]
        self.me.rotation.data = [car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll]

        temp = Vector3([car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z])
        
        self.me.matrix = rotator_to_matrix(self.me)
        self.me.rvelocity.data = [temp*self.me.matrix[1],temp*self.me.matrix[2],temp*self.me.matrix[0]]
        
        self.me.boost = car.boost
        self.me.grounded = car.has_wheel_contact

        ball = game.game_ball.physics
        self.ball.location.data = [ball.location.x, ball.location.y, ball.location.z]
        self.ball.velocity.data = [ball.velocity.x, ball.velocity.y, ball.velocity.z]
        self.ball.rotation.data = [ball.rotation.pitch, ball.rotation.yaw, ball.rotation.roll]
        self.ball.rvelocity.data = [ball.angular_velocity.x, ball.angular_velocity.y, ball.angular_velocity.z]
        

        self.ball.local_location = to_local(self.ball,self.me)

        #collects info for all other cars in match, updates objects in self.players accordingly
        for i in range(game.num_cars):
            if i != self.index:
                car = game.game_cars[i]
                temp = obj()
                temp.index = i
                temp.team = car.team
                temp.location.data = [car.physics.location.x, car.physics.location.y, car.physics.location.z]
                temp.velocity.data = [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]
                temp.rotation.data = [car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll]
                #temps = Vector3([car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z])
                #temp.matrix = rotator_to_matrix(temp)
                #temp.rvelocity.data = [temps*temp.matrix[1],temps*temp.matrix[2],temps*temp.matrix[0]]
                temp.boost = car.boost
                flag = False
                for item in self.players:
                    if item.index == i:
                        item = temp
                        flag = True
                        break
                if not flag:
                    self.players.append(temp)

                

            
                

                

