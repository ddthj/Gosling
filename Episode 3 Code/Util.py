import math

GOAL_WIDTH = 1900
FIELD_LENGTH = 10280
FIELD_WIDTH = 8240

class Vector3:
    def __init__(self, data):
        self.data = data
    def __sub__(self,value):
        return Vector3([self.data[0]-value.data[0],self.data[1]-value.data[1],self.data[2]-value.data[2]])
    def __mul__(self,value):
        return (self.data[0]*value.data[0] + self.data[1]*value.data[1] + self.data[2]*value.data[2])

class obj:
    def __init__(self):
        self.location = Vector3([0,0,0])
        self.velocity = Vector3([0,0,0])
        self.rotation = Vector3([0,0,0])
        self.rvelocity = Vector3([0,0,0])

        self.local_location = Vector3([0,0,0])
        self.boost = 0

def to_local(target_object,our_object):
    x = (toLocation(target_object) - our_object.location) * our_object.matrix[0]
    y = (toLocation(target_object) - our_object.location) * our_object.matrix[1]
    z = (toLocation(target_object) - our_object.location) * our_object.matrix[2]
    return Vector3([x,y,z])

def rotator_to_matrix(our_object):
    r = our_object.rotation.data
    CR = math.cos(r[2])
    SR = math.sin(r[2])
    CP = math.cos(r[0])
    SP = math.sin(r[0])
    CY = math.cos(r[1])
    SY = math.sin(r[1])

    matrix = []
    matrix.append(Vector3([CP*CY, CP*SY, SP]))
    matrix.append(Vector3([CY*SP*SR-CR*SY, SY*SP*SR+CR*CY, -CP * SR]))
    matrix.append(Vector3([-CR*CY*SP-SR*SY, -CR*SY*SP+SR*CY, CP*CR]))
    return matrix

def sign(x):
    if x <= 0:
        return -1
    else:
        return 1

def angle2D(target_location,object_location):
    difference = toLocation(target_location) - toLocation(object_location)
    return math.atan2(difference.data[1], difference.data[0])


def velocity2D(target_object):
    return math.sqrt(target_object.velocity.data[0]**2 + target_object.velocity.data[1]**2)

def toLocal(target,our_object):
    if isinstance(target,obj):
        return target.local_location
    else:
        return to_local(target,our_object)

def toLocation(target):
    if isinstance(target,Vector3):
        return target
    elif isinstance(target,list):
        return Vector3(target)
    else:
        return target.location

def distance2D(target_object, our_object):
    difference = toLocation(target_object) - toLocation(our_object)
    return math.sqrt(difference.data[0]**2 + difference.data[1]**2)

