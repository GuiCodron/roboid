"Simulation File"
import sys
import random
import pygame
import numpy as np
from Boid import Boid
from HerdManager import HerdManager
from Particle import Goal, Obstacle, TYPE_GOAL
from params import WIDTH, HEIGHT, FRAME_RATE, BOIDS_NUMBER, OBSTACLE_NUMBER, GOAL_NUMBER



def rand_boid_generation(id):
    return Boid(id, pos=np.random.randint(800, size=2), speed=np.random.randint(-Boid.v_max, high=Boid.v_max, size=2), acc=np.random.randint(-Boid.acc_max, high=Boid.acc_max, size=2))

Boid.max_height = HEIGHT
Boid.max_width = WIDTH

def run():
    global FRAME_RATE
    herd = HerdManager(WIDTH, HEIGHT)
    colors = dict()

    pygame.init()
    myfont = pygame.font.SysFont("monospace", 15)
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    goal = Goal(0)
    obstacle = Obstacle(0)
    colors[goal.get_key()] = np.array((255, 0, 0))
    colors[obstacle.get_key()] = np.zeros(3)
    boid_id = 0
    obstacle_id = 1
    goal_id = 1
    for i in range(OBSTACLE_NUMBER):
        new_obstacle = Obstacle(obstacle_id, pos=np.random.randint(WIDTH, size=2))
        herd.add_element(new_obstacle)
        obstacle_id += 1
    for i in range(GOAL_NUMBER):
        new_goal = Goal(goal_id, pos=np.random.randint(WIDTH, size=2))
        herd.add_element(new_goal)
        goal_id += 1
    for i in range(BOIDS_NUMBER):
        new_boid = rand_boid_generation(boid_id)
        herd.add_element(new_boid)
        boid_id += 1
        new_boid.set_goal(herd.elements[Boid.gen_key(TYPE_GOAL, random.randint(1, GOAL_NUMBER))])
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
                    boid_id += 1
                    new_boid.set_goal(herd.elements[Boid.gen_key(TYPE_GOAL, random.randint(1, GOAL_NUMBER))])
                if (event.unicode) == 'm':
                    FRAME_RATE -= 1
                if (event.unicode) == 'n':
                    FRAME_RATE += 1
            if event.type == pygame.MOUSEMOTION:
                goal.pos = np.array(event.pos)
                obstacle.pos = np.array(event.pos)

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    herd.add_element(goal)
                if event.button == 3:
                    herd.add_element(obstacle)
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    herd.remove_element(goal.get_key())
                if event.button == 3:
                    herd.remove_element(obstacle.get_key())

        screen.fill((255, 255, 255))
        for key, element in herd.elements.items():
            pygame.draw.lines(screen, (255, 0, 0), False, [element.get_pos(), element.get_pos() + element.speed], 2)
            pygame.draw.circle(screen, element.color, element.get_pos(), element.size, 0)
            #pygame.draw.circle(screen, (255, 0, 0), element.get_pos(), element.rejection_radius[1], 1)
            #acc = np.random.randint(200, size=2) - 100
            #boid.set_acc(acc)
        herd.update()
        label = myfont.render(str(int(clock.get_fps())) + " fps", 1, (0,0,0))
        screen.blit(label, (720, 2))
        pygame.display.update()
        msElapsed = clock.tick(FRAME_RATE)

if __name__ == '__main__':
    run()
