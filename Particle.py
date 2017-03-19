import math
import numpy as np
from params import WIDTH, HEIGHT

TYPE_PARTICLE = 0
TYPE_BOID = 1
TYPE_GOAL = 2
TYPE_OBSTACLE = 3
class Particle:
    "General class to describe a Particle of the map"
    rejection_radius = (0, 1)
    confort_radius = (0, 0)
    attraction_radius = (0, 0)

    def __init__(self, identity, pos=np.zeros(2), speed=np.zeros(2), acc=np.zeros(2), size=6, color=np.zeros(3)):
        self.identity = identity
        self.type = TYPE_PARTICLE
        self.pos = pos
        self.speed = speed
        self.acc = acc
        self.size = size
        self.color = color
        self.vectors = np.zeros(4)

    def update(self):
        "Upate function required for all types of elements"
        pass

    def get_key(self):
        "Unique identifier for element"
        return str(self.type) + ' ' + str(self.identity)

    def distance(self, element):
        "Distance from self to element"
        diff = element.pos - self.pos
        return math.sqrt(np.sum(diff ** 2))
        #enabled this piece of code to make the map like a tore
        if diff[0] > WIDTH / 2:
            diff[0] = WIDTH - element.pos[0] - self.pos[0]
        if diff[1] > HEIGHT / 2:
            diff[1] = HEIGHT - element.pos[1] - self.pos[1]

    def get_pos(self):
        "Return position as int32"
        return self.pos.astype('int32')
    @staticmethod
    def gen_key(elem_type, elem_id):
        "Generate key of a certain type and id"
        return str(elem_type) + ' ' + str(elem_id)

    def reaction(self, herd):
        "Compute the reaction of the particule to it's surrounding"
        pass

class Goal(Particle):
    "Symbolize the point some particle must go to"

    def __init__(self, identity, pos=np.zeros(2), size=12, color=None):
        Particle.__init__(self, identity, pos, size=size)
        self.color = color or np.random.randint(255,size=3)
        self.type = TYPE_GOAL

class Obstacle(Particle):
    "Symbolize obstacle on the map"
    rejection_radius = (0, 100)

    def __init__(self, identity, pos=np.zeros(2), size=10, color=np.array((0, 0, 0))):
        Particle.__init__(self, identity, pos, size=size, color=color)
        self.type = TYPE_OBSTACLE

