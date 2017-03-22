import numpy as np
import pandas as pd
import math
import datetime 
from Boid import Boid
from Particle import TYPE_BOID, TYPE_GOAL, TYPE_OBSTACLE
from params import GOAL_NUMBER, BOIDS_NUMBER, OBSTACLE_NUMBER, WIDTH, HEIGHT, USER_GOAL

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
        self.elements = {TYPE_GOAL:{}, TYPE_BOID:{}, TYPE_OBSTACLE:{}}
        self.collision_count = 0
        self.collisions = []
        self.goal_counter = [[[-1] for j in range(GOAL_NUMBER)] for i in range(GOAL_NUMBER)]
        self.collision_counter = [[0 for j in range(GOAL_NUMBER)] for i in range(GOAL_NUMBER)]
        self.creation_date = datetime.datetime.now()
        print(self.creation_date)

    def update(self):
        "Update all the area to get next iteration"
        new_area = np.zeros(self.area.shape)
        self.collisions = []
        for el_type, elements in self.elements.items():
            for key, element in elements.items():
                element.update()
                new_area[element.get_pos() % [WIDTH, HEIGHT], :] = [element.identity, element.type]

        for key, element in self.elements[TYPE_BOID].items():
            element.reaction(self)
            #if element.type == TYPE_BOID:
            #    element.set_acc(self.boid_reaction(element))

        self.area = new_area.copy()

    def add_element(self, element):
        "Add new element to herd manager"
        self.elements[element.type][element.identity] = element
        self.area[element.get_pos(), :] = [element.identity, element.type]
        if element.type == TYPE_GOAL and element.identity != USER_GOAL:
            for counter in self.goal_counter:
                counter.append([-1])
            self.goal_counter.append([[-1] for j in range(len(self.elements[TYPE_GOAL]))])
            for counter in self.collision_counter:
                counter.append(0)
            self.collision_counter.append([0 for j in range(len(self.elements[TYPE_GOAL]))])


    def remove_element(self, type, key):
        "Add new element to herd manager"
        self.area[self.elements[type][key].get_pos(), :] = [0, 0]
        if type == TYPE_GOAL and key != USER_GOAL:
            for counter in self.goal_counter:
                counter.pop()
            self.goal_counter.pop()
            for counter in self.collision_counter:
                counter.pop()
            self.collision_counter.pop()
        del self.elements[type][key]

    def goal_reached(self, last_id, new_id, time, collisions):
        "Update stats of the herd"
        self.collision_counter[last_id][new_id] += collisions
        if self.goal_counter[last_id][new_id][0] == -1:
            self.goal_counter[last_id][new_id][0] = time
        else:
            self.goal_counter[last_id][new_id].append(time)
        return

    def print_stats(self, save=True):
        "Compute and print the dataframe showing the statistics of the current run"
        columns = ['Meilleur temps', 'Moyenne', 'Variance', 'Minimum', 'Maximum', 'Collisions', 'Nb Echantillons']
        nb_goals, nb_obstacles, nb_boids = (len(self.elements[TYPE_GOAL]),
                                            len(self.elements[TYPE_OBSTACLE]),
                                            len(self.elements[TYPE_BOID]))
        datas = np.zeros((nb_goals, nb_goals, len(columns)))
        index = []
        for i in range(nb_goals):
            for j in range(nb_goals):
                index.append(str(i)+ "->" + str(j))
                datas[i, j, 0] = self.elements[TYPE_GOAL][i].distance(self.elements[TYPE_GOAL][j]) / Boid.v_max
                datas[i, j, 1] = np.mean(self.goal_counter[i][j])
                datas[i, j, 2] = math.sqrt(np.var(self.goal_counter[i][j]))
                datas[i, j, 3] = np.min(self.goal_counter[i][j])
                datas[i, j, 4] = np.max(self.goal_counter[i][j])
                datas[i, j, 5] = self.collision_counter[i][j]
                datas[i, j, 6] = len(self.goal_counter[i][j])
        idx = pd.Index(index)
        duration = (datetime.datetime.now() - self.creation_date).total_seconds()
        data_frame = pd.DataFrame(datas.reshape((nb_goals * nb_goals, len(columns))), columns=columns, index=idx)
        print(data_frame)
        parameters_head = ["Nombre de boids", "Nombre d'objectifs", "Nomber d'obstacles", "Largeur", "Hauteur",
                           "Nombre de collisions", "Durée du test", "Vitesse maximale"]
        parameters = np.array([nb_boids, nb_goals, nb_obstacles, WIDTH, HEIGHT, self.collision_count, duration, Boid.v_max]).reshape((1,8))
        params_frame = pd.DataFrame(parameters, columns=parameters_head, index=[' '])
        print(params_frame)
        if save:
            creation_date = self.creation_date.strftime("%d-%m-%Y_%H:%M")
            data_frame.to_csv('./statistics/' + creation_date +'_S_' + str(int(duration))+ '_G_' + str(nb_goals) + '_B_' + str(nb_boids) + '_O_' + str(nb_obstacles) + '.csv')
            params_frame.to_csv('./statistics/' + creation_date +'_S_' + str(int(duration))+ '_G_' + str(nb_goals) + '_B_' + str(nb_boids) + '_O_' + str(nb_obstacles) + '.csv', mode='a')

if __name__ == '__main__':
    herd = HerdManager(800, 800)
    boid = Boid(0)
    herd.add_element(boid)
    boid2 = Boid(1,pos=np.array([50, 50]))
    herd.add_element(boid2)
    herd.update()
