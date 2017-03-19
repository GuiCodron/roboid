import numpy as np
import pandas as pd
import math
import datetime 
from Boid import Boid
from Particle import TYPE_BOID, TYPE_GOAL
from params import GOAL_NUMBER, BOIDS_NUMBER, OBSTACLE_NUMBER

DESCRIPTOR_LENGTH = 2 #number of descriptor for 1 cell (id, type)

def create_data(elem):
    val = np.zeros(5)
    try:
        val[0] = np.mean(elem)
        val[1] = np.var(elem)
        val[2] = np.min(elem)
        val[3] = np.max(elem)
        val[4] = len(elem)
    except:
        pass
    return val

class HerdManager():
    "Representation of all the elements has a map, manage elements update"
    def __init__(self, width, height):
        self.area = np.zeros((width, height, DESCRIPTOR_LENGTH), dtype='int32')
        self.elements = dict()
        self.collision_counter = 0
        self.collisions = []
        self.goal_counter = [[[-1] for j in range(GOAL_NUMBER)] for i in range(GOAL_NUMBER)]
        self.creation_date = datetime.datetime.now()
        print(self.creation_date)

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
        if self.goal_counter[last_id][new_id][0] == -1:
            self.goal_counter[last_id][new_id][0] = time
        else:
            self.goal_counter[last_id][new_id].append(time)
        return
        if pattern not in self.goal_counter:
            self.goal_counter[pattern] = {'mean':0, 'var':0, 'min':0, 'max':0, 'values':np.empty((1,), dtype='int32')}
        goal_pat = self.goal_counter[pattern]
        goal_pat['values'] = np.append(goal_pat['values'], time)
        goal_pat['mean'] = np.mean(goal_pat['values'])
        goal_pat['var'] = np.var(goal_pat['values'])
        goal_pat['min'] = np.min(goal_pat['values'])
        goal_pat['max'] = np.max(goal_pat['values'])
        print(self.goal_counter)

    def print_stats(self, save=True):
        "Compute and print the dataframe showing the statistics of the current run"
        datas = np.zeros((GOAL_NUMBER, GOAL_NUMBER, 6))
        index = []
        for i in range(GOAL_NUMBER):
            for j in range(GOAL_NUMBER):
                index.append(str(i)+ "->" + str(j))
                datas[i, j, 0] = self.elements[Boid.gen_key(TYPE_GOAL, i)].distance(self.elements[Boid.gen_key(TYPE_GOAL, j)]) / Boid.v_max
                datas[i, j, 1] = np.mean(self.goal_counter[i][j])
                datas[i, j, 2] = math.sqrt(np.var(self.goal_counter[i][j]))
                datas[i, j, 3] = np.min(self.goal_counter[i][j])
                datas[i, j, 4] = np.max(self.goal_counter[i][j])
                datas[i, j, 5] = len(self.goal_counter[i][j])
        idx = pd.Index(index)
        data_frame = pd.DataFrame(datas.reshape((GOAL_NUMBER*GOAL_NUMBER, 6)), columns=['Meilleur temps', 'Moyenne', 'Variance', 'Minimum', 'Maximum', 'Nb Echantillons'], index=idx)
        if save:
            duration = (datetime.datetime.now() - self.creation_date).total_seconds()
            creation_date = self.creation_date.strftime("%d-%m-%Y_%H:%M")
            data_frame.to_csv('./statistics/' + creation_date +'_S_' + str(int(duration))+ '_G_' + str(GOAL_NUMBER) + '_B_' + str(BOIDS_NUMBER) + '_O_' + str(OBSTACLE_NUMBER) + '.csv')

if __name__ == '__main__':
    herd = HerdManager(800, 800)
    boid = Boid(0)
    herd.add_element(boid)
    boid2 = Boid(1,pos=np.array([50, 50]))
    herd.add_element(boid2)
    herd.update()
