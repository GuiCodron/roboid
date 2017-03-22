
from math import cos, sin, sqrt, pi, atan
import numpy as np
import random
from params import GOAL_NUMBER, USER_GOAL, V_MAX, ACC_MAX, MULT_SPEED
from Particle import Particle, TYPE_BOID, TYPE_GOAL
from model.model3 import bot

class Boid(Particle):
    "Boid: particle that move according to the hord around it"
    rejection_radius = (0, V_MAX * 4)
    confort_radius = (40, 100)
    #attraction_radius = (100, 200)
    v_max = V_MAX
    acc_max = ACC_MAX
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
        self.last_goal_id = None
        self.color = color or np.random.randint(255, size=3)
        self.timer = 0
        self.collision_count = 0
        self.bot = bot()
        self.bot.posx = self.pos[0] / self.v_max * 0.74
        self.bot.posy = self.pos[1] / self.v_max * 0.74

    def set_acc(self, acc):
        "Set acceleration"
        self.acc = (self.pos + acc) / MULT_SPEED
        return acc
        #old_code
        norm = sqrt(np.sum(acc ** 2))
        if norm > self.acc_max:
            self.acc = (acc * self.acc_max) / norm
        else:
            self.acc = acc
        return self.acc

    def update(self):
        "update boids position and speed"
        old_pos = np.array([self.bot.posx, self.bot.posy])
        self.bot.movebot(self.acc[0], self.acc[1])
        self.speed = (np.array([self.bot.posx, self.bot.posy]) - old_pos) * MULT_SPEED
        self.pos = np.array([self.bot.posx, self.bot.posy]) * MULT_SPEED
        return
        #old code
        new_speed = 0.3 * self.speed + self.acc
        norm = sqrt(np.sum(new_speed ** 2))
        if norm > self.v_max:
            self.speed = new_speed * self.v_max / norm
        else:
            self.speed = new_speed
        self.pos = np.mod((self.pos + self.speed), (self.max_width, self.max_height))

    def set_goal(self, goal):
        "Set new goal for the boid"
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
        self.timer += 1
        width, height, depth = herd.area.shape
        for el_type, elements in herd.elements.items():
            #cas du goal
            if el_type == TYPE_GOAL:               #Attraction to goal
                if USER_GOAL in elements:
                    element = elements[USER_GOAL]
                elif self.goal_id in elements:
                    element = elements[self.goal_id]
                else:
                    continue
                dist = self.distance(element)
                goal_attraction = ((element.pos - self.pos) / 10) if 3 * dist < Boid.v_max else ((element.pos - self.pos) * Boid.v_max / dist)
                if dist <= self.size + element.size and self.goal_id == element.identity:
                    #print("goal_reached")
                    if self.last_goal_id is not None:
                        herd.goal_reached(self.last_goal_id, self.goal_id, self.timer, self.collision_count)
                    self.timer = 0
                    self.collision_count = 0
                    self.last_goal_id, self.goal_id = (self.goal_id, random.randint(0, len(elements)-1)) # new goal choosen at random
                    self.set_goal(elements[self.goal_id])
                continue
            for element in elements.values():
                if element == self:
                    continue

                dist = self.distance(element)
                if dist < (element.size + self.size) / 2:
                    herd.collision_count += 1
                    herd.collisions.append(self.get_pos())
                    self.collision_count += 1
                if dist <= element.attraction_radius[1]:    #process attaction to other entity
                    if element.attraction_radius[0] <= dist:
                        attraction += element.pos - self.pos
                        attraction_count += 1
                    velocity_matching += element.speed      #velocity matching if close to other
                    close_count += 1
                if element.rejection_radius[0] <= dist <= element.rejection_radius[1]:#process rejection to other entity
                    rejection -= (element.rejection_radius[1] - dist) * (element.pos - self.pos) / dist / 2 if dist != 0 else np.random.randint(160, size=2)
        if attraction_count != 0:
            attraction /= attraction_count * 100
        if close_count != 0:
            velocity_matching = (velocity_matching/close_count - self.speed) / 30
        #print(boid.identity, 'number of neighboor: ', attraction_count, 'attraction', attraction, 'velocity_matching', velocity_matching, 'rejection', rejection)
        #print(velocity_matching + attraction + rejection)

        self.vectors = np.array((velocity_matching, attraction, rejection, goal_attraction))
        return self.set_acc(velocity_matching + attraction + rejection + goal_attraction + np.random.random_sample((2,)) * Boid.v_max / 10)


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

