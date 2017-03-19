
from math import cos, sin, sqrt, pi, atan
import numpy as np
import random
from params import GOAL_NUMBER
from Particle import Particle, TYPE_BOID, TYPE_GOAL

class Boid(Particle):
    "Boid: particle that move according to the hord around it"
    rejection_radius = (0, 60)
    confort_radius = (40, 100)
    #attraction_radius = (100, 200)
    v_max = 20
    acc_max = 10
    max_width = 100
    max_height = 100

    def __init__(self, identity, pos=np.zeros(2), speed=np.zeros(2), acc=np.zeros(2), goal=None, color=None):
        "init boid with id, initial position, speed and acceleration"
        Particle.__init__(self, identity)
        self.type = TYPE_BOID
        self.pos = pos.astype('int32')
        self.speed = speed.astype('int32')
        self.acc = acc.astype('int32')
        self.goal_id = goal
        self.color = color or np.random.randint(255, size=3)

    def set_acc(self, acc):
        "Set acceleration"
        norm = sqrt(np.sum(acc ** 2))
        if norm > self.acc_max:
            self.acc = (acc * self.acc_max) / norm
        else:
            self.acc = acc
        return self.acc

    def update(self):
        "update boids position and speed"
        new_speed = self.speed + self.acc
        norm = sqrt(np.sum(new_speed ** 2))
        if norm > self.v_max:
            self.speed = new_speed * self.v_max / norm
        else:
            self.speed = new_speed
        self.pos = np.mod((self.pos + self.speed), (self.max_width, self.max_height))

    def set_goal(self, goal):
        self.goal_id = goal.identity
        self.color = goal.color


    def reaction(self, herd):
        "reaction to his surrounding"
        velocity_matching = np.zeros(2)
        attraction = np.zeros(2)
        goal_attraction = np.zeros(2)
        attraction_count = 0
        close_count = 0
        rejection = np.zeros(2)
        width, height, depth = herd.area.shape
        for key, element in herd.elements.items():
            if element == self:
                continue

            dist = self.distance(element)
            if element.type == TYPE_GOAL:               #Attraction to goal
                if element.identity == 0:
                    goal_attraction = (element.pos - self.pos) / 10
                elif element.identity == self.goal_id and not np.any(goal_attraction):
                    goal_attraction = (element.pos - self.pos) / 10
                    if dist <= self.size + element.size:
                        #print("goal_reached")
                        self.set_goal(herd.elements[self.gen_key(TYPE_GOAL, random.randint(1, GOAL_NUMBER))])
                continue
            if dist < (element.size + self.size) / 2:
                herd.collision_counter += 1
                print("Number of collision : ", herd.collision_counter)
            if dist <= element.attraction_radius[1]:    #process attaction to other entity
                if element.attraction_radius[0] <= dist:
                    attraction += element.pos - self.pos
                    attraction_count += 1
                velocity_matching += element.speed      #velocity matching if close to other
                close_count += 1
            if element.rejection_radius[0] <= dist <= element.rejection_radius[1]:#process rejection to other entity
                rejection -= (element.rejection_radius[1] - dist) * (element.pos - self.pos) / dist if dist != 0 else np.random.randint(160, size=2)
        if attraction_count != 0:
            attraction /= attraction_count * 100
        if close_count != 0:
            velocity_matching = (velocity_matching/close_count - self.speed) / 30
        #print(boid.identity, 'number of neighboor: ', attraction_count, 'attraction', attraction, 'velocity_matching', velocity_matching, 'rejection', rejection)
        #print(velocity_matching + attraction + rejection)

        return self.set_acc(velocity_matching + attraction + rejection + goal_attraction)


    def get_acceleration_cone(self):
        "Give access to the range of movement the boids has access to"
        if self.speed[1] != 0:
            angle = atan(self.speed[0]/ self.speed[1])
        else:
            angle = 0.0 if self.speed[0] >= 0 else pi
        rot = np.array([cos(angle), -sin(angle)],
                       [sin(angle), cos(angle)],
                       dtype='float64')
        result = np.array([[cos(self.speed/ self.v_max), sin(self.speed/ self.v_max)],
                           [cos(self.speed/ self.v_max), -sin(self.speed/ self.v_max)]])
        result[0, :] = np.dot(rot, result[0, :])
        result[1, :] = np.dot(rot, result[1, :])
        return result

if __name__ == '__main__':
    boid = Boid(1)
    boid.get_acceleration_cone()

