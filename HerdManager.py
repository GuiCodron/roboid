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

if __name__ == '__main__':
    herd = HerdManager(800, 800)
    boid = Boid(0)
    herd.add_element(boid)
    boid2 = Boid(1,pos=np.array([50, 50]))
    herd.add_element(boid2)
    herd.update()
