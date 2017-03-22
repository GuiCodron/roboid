"Simulation File"
import sys
import random
from math import floor, ceil, sqrt, cos, sin
import pygame
import numpy as np
from Boid import Boid
from HerdManager import HerdManager
from Particle import Goal, Obstacle, TYPE_GOAL, TYPE_BOID, distance
from params import WIDTH, HEIGHT, FRAME_RATE, BOIDS_NUMBER, OBSTACLE_NUMBER, GOAL_NUMBER, TRACK_COLLISION, USER_GOAL
import atexit



def rand_boid_generation(id):
    "Create a boid at a random place with random color, acceleration and speed"
    pos = np.array([WIDTH * 2/3, HEIGHT * 2/3]) * np.random.random_sample((2,)) + np.array([WIDTH * 1/6, HEIGHT * 1/6])
    return Boid(id, pos=random_pos(WIDTH, HEIGHT), speed=np.random.randint(-Boid.v_max, high=Boid.v_max, size=2), acc=np.random.randint(-Boid.acc_max, high=Boid.acc_max,               size=2))

def random_pos(width, height):
    "generate random position on screen"
    return np.array([width, height]) * np.random.random_sample((2,))

Boid.max_height = HEIGHT
Boid.max_width = WIDTH

def run():
    "Run full simulation"
    global FRAME_RATE
    global BOIDS_NUMBER
    global GOAL_NUMBER
    herd = HerdManager(WIDTH, HEIGHT)
    #atexit.register(herd.print_stats)
    colors = dict()

    pygame.init()
    mouse_pos = np.zeros(2)
    playing = True
    radius_display = False
    myfont = pygame.font.SysFont("monospace", 15)
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    goal = Goal(USER_GOAL)
    obstacle = Obstacle(USER_GOAL)
    colors[goal.get_key()] = np.array((255, 0, 0))
    colors[obstacle.get_key()] = np.zeros(3)
    boid_id = 0
    obstacle_id = 0
    goal_id = 0
    for i in range(OBSTACLE_NUMBER):
        #pos = np.array([5 * WIDTH / 12 +  WIDTH / 3 / OBSTACLE_NUMBER * i, HEIGHT / 2])
        pos = np.array(random_pos(WIDTH, HEIGHT))
        new_obstacle = Obstacle(obstacle_id, pos=pos)
        herd.add_element(new_obstacle)
        obstacle_id += 1
    w_nb_r, h_nb_r = (sqrt(GOAL_NUMBER * WIDTH / HEIGHT), sqrt(GOAL_NUMBER * HEIGHT / WIDTH))
    w_nb, h_nb = (ceil(w_nb_r), round(h_nb_r)) if WIDTH >= HEIGHT else (round(w_nb_r), ceil(h_nb_r))
    w, h = (WIDTH / w_nb / 2, HEIGHT / h_nb / 2)
    for i in range(GOAL_NUMBER):
        new_goal = Goal(goal_id, pos=np.array([w + w * 2 * (i % w_nb), h + h * 2 * int(i / w_nb)]))
        herd.add_element(new_goal)
        goal_id += 1
    for i in range(BOIDS_NUMBER):
        new_boid = rand_boid_generation(boid_id)
        herd.add_element(new_boid)
        boid_id += 1
        new_boid.set_goal(herd.elements[TYPE_GOAL][random.randint(0, len(herd.elements[TYPE_GOAL]) - 1)])
    while True:
    # check for quit events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            #print(event)
            if event.type == pygame.KEYDOWN:
                if (event.unicode) == 'a':
                    new_boid = rand_boid_generation(boid_id)
                    herd.add_element(new_boid)
                    BOIDS_NUMBER += 1
                    boid_id += 1
                    new_boid.set_goal(herd.elements[TYPE_GOAL][random.randint(0, len(herd.elements[TYPE_GOAL]) - 1)])
                if (event.unicode) == 'm':
                    FRAME_RATE -= 1
                if (event.unicode) == 'n':
                    FRAME_RATE += 1
                if (event.unicode) == 'o':
                    herd.add_element(Obstacle(obstacle_id, pos=mouse_pos))
                    obstacle_id += 1
                if (event.unicode) == 't':
                    radius_display = not radius_display
                if (event.unicode) == 'g':
                    herd.add_element(Goal(goal_id, pos=mouse_pos))
                    goal_id += 1
                if (event.unicode) == ' ':
                    herd.collisions = []
                if (event.unicode) == 's':
                    herd.print_stats(save=True)
                if (event.key) == 112: # p key for pause
                    herd.print_stats(save=False)
                    playing = False
                if (event.key) == 27: # exit on esc
                    pygame.quit()
                    sys.exit()
            if event.type == pygame.KEYUP:
                if (event.key) == 112: #p key for pause
                    playing = True

            if event.type == pygame.MOUSEMOTION:
                mouse_pos = np.array(event.pos)
                goal.pos = mouse_pos
                obstacle.pos = mouse_pos

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    herd.add_element(goal)
                if event.button == 3:
                    herd.add_element(obstacle)
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    herd.remove_element(TYPE_GOAL, USER_GOAL)
                if event.button == 3:
                    herd.remove_element(obstacle.type, USER_GOAL)

        screen.fill((255, 255, 255))
        #display elements on screen
        for el_list in herd.elements.values():
            for element in el_list.values():
                if element.type == TYPE_BOID:
                    for i in range(element.vectors.shape[0]):
                        col = (255, 255, 255) if i <= 1 else ((0, 0, 0) if i == 2 else (0, 255, 0))
                        pygame.draw.lines(screen, col, False, [element.get_pos(), element.get_pos() + element.vectors[i].astype('int32')], 2)

                    pygame.draw.lines(screen, (255, 0, 0), False, [element.get_pos(),
                                                                   element.get_pos() + element.speed], 2)
                
                pygame.draw.circle(screen, element.color, element.get_pos(), element.size, 0)
                if radius_display:
                    pygame.draw.circle(screen, element.color, element.get_pos(), element.rejection_radius[1], min(element.rejection_radius[1], 1))
                if element.type == TYPE_GOAL:
                    pygame.draw.circle(screen, element.color, element.get_pos(), element.size, 0)
                    goal_id_text = myfont.render(str(element.identity), 2, (0, 0, 0))
                    screen.blit(goal_id_text, element.get_pos() - [element.size / 2, element.size / 2])


        if len(herd.collisions) > 0:
            for collison in herd.collisions:
                pygame.draw.circle(screen, (255,0,0), collison, 15, 3)
        # stop update if there was a collision and we are tracking it
        if playing and (len(herd.collisions) == 0 or not TRACK_COLLISION): 
            herd.update()
        label = myfont.render(str(int(clock.get_fps())) + " fps", 2, (0, 0, 0))
        screen.blit(label, (WIDTH - 80, 2))
        collision = myfont.render("Collisions : "+str(int(herd.collision_count)), 2, (0, 0, 0))
        screen.blit(collision, (20, 2))
        goals_nb = myfont.render("Nombre de boids : " + str(GOAL_NUMBER), 2, (0, 0, 0))
        screen.blit(goals_nb, (20, 22))
        boids_nb = myfont.render("Nombre de d'objectifs : " + str(BOIDS_NUMBER), 2, (0, 0, 0))
        screen.blit(boids_nb, (20, 42))
        pygame.display.update()
        msElapsed = clock.tick(FRAME_RATE)

if __name__ == '__main__':
    run()
