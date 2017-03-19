import numpy as np
from Boid import Boid
from Particle import TYPE_BOID, TYPE_GOAL

DESCRIPTOR_LENGTH = 2 #number of descriptor for 1 cell (id, type)

class HerdManager():
    "Representation of all the elements has a map, manage elements update"
    def __init__(self, width, height):
        self.area = np.zeros((width, height, DESCRIPTOR_LENGTH), dtype='int32')
        self.elements = dict()
        self.collision_counter = 0
        self.collisions = []
        self.goal_counter = dict()

    def update(self):
        "Update all the area to get next iteration"
        new_area = np.zeros(self.area.shape)
        self.collisions = []
        for key, element in self.elements.items():
            element.update()
            new_area[element.get_pos(), :] = [element.identity, element.type]

        for key, element in self.elements.items():
            element.reaction(self)
            #if element.type == TYPE_BOID:
            #    element.set_acc(self.boid_reaction(element))

        self.area = new_area.copy()

    def add_element(self, element):
        "Add new element to herd manager"
        self.elements[element.get_key()] = element
        self.area[element.get_pos(), :] = [element.identity, element.type]

    def remove_element(self, key):
        "Add new element to herd manager"
        self.area[self.elements[key].pos, :] = [0,0]
        del self.elements[key]

    def goal_reached(self, last_id, new_id, time):
        "Give access to stats of the herd"
        pattern = str(last_id)+ " " + str(new_id)
        if pattern not in self.goal_counter:
            self.goal_counter[pattern] = {'mean':0, 'var':0, 'min':0, 'max':0, 'values':np.empty((1,), dtype='int32')}
        goal_pat = self.goal_counter[pattern]
        goal_pat['values'] = np.append(goal_pat['values'], time)
        goal_pat['mean'] = np.mean(goal_pat['values'])
        goal_pat['var'] = np.var(goal_pat['values'])
        goal_pat['min'] = np.min(goal_pat['values'])
        goal_pat['max'] = np.max(goal_pat['values'])
        print(self.goal_counter)


if __name__ == '__main__':
    herd = HerdManager(800, 800)
    boid = Boid(0)
    herd.add_element(boid)
    boid2 = Boid(1,pos=np.array([50, 50]))
    herd.add_element(boid2)
    herd.update()
