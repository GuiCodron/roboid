"Simulation File"
import sys
import random
import pygame
import numpy as np
from Boid import Boid
from HerdManager import HerdManager
from Particle import Goal, Obstacle, TYPE_GOAL
from params import WIDTH, HEIGHT, FRAME_RATE, BOIDS_NUMBER, OBSTACLE_NUMBER, GOAL_NUMBER, TRACK_COLLISION



def rand_boid_generation(id):
    "Create a boid at a random place with random color, acceleration and speed"
    pos = np.array([WIDTH * 2/3, HEIGHT * 2/3]) * np.random.random_sample((2,)) + np.array([WIDTH * 1/6, HEIGHT * 1/6])
    return Boid(id, pos=random_pos(WIDTH, HEIGHT), speed=np.random.randint(-Boid.v_max, high=Boid.v_max, size=2), acc=np.random.randint(-Boid.acc_max, high=Boid.acc_max, size=2))

def random_pos(width, height):
    "generate random position in screen"
    return np.array([width, height]) * np.random.random_sample((2,))

Boid.max_height = HEIGHT
Boid.max_width = WIDTH

def run():
    global FRAME_RATE
    herd = HerdManager(WIDTH, HEIGHT)
    colors = dict()

    pygame.init()
    playing = True
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
        new_obstacle = Obstacle(obstacle_id, pos=random_pos(WIDTH, HEIGHT))
        herd.add_element(new_obstacle)
        obstacle_id += 1
    for i in range(GOAL_NUMBER):
        new_goal = Goal(goal_id, pos=random_pos(WIDTH, HEIGHT))
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
                if (event.unicode) == ' ':
                    herd.collisions = []
                if (event.key) == 112: # p key for pause
                    playing = False
                if (event.key) == 27: # exit on esc
                    pygame.quit()
                    sys.exit()
            if event.type == pygame.KEYUP:
                if (event.key) == 112: #p key for pause
                    playing = True

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
        for key, element in herd.elements.items():                          #display elements on screen
            for i in range(element.vectors.shape[0]):
                col = (255, 255, 255) if i <= 1 else ((0, 0, 0) if i == 2 else (0, 255, 0))
                pygame.draw.lines(screen, col, False, [element.get_pos(), element.get_pos() + element.vectors[i].astype('int32')], 2)
            pygame.draw.lines(screen, (255, 0, 0), False, [element.get_pos(), element.get_pos() + element.speed], 2)
            pygame.draw.circle(screen, element.color, element.get_pos(), element.size, 0)
            if element.type == TYPE_GOAL:
                goal_id = myfont.render(str(element.identity), 2, (0, 0, 0))
                screen.blit(goal_id, element.get_pos() - [element.size / 2, element.size / 2])

        if len(herd.collisions) > 0:
            for collison in herd.collisions:
                pygame.draw.circle(screen, (255,0,0), collison, 15, 3)
        if playing and (len(herd.collisions) == 0 or not TRACK_COLLISION): # stop update if there was a collision and we are tracking it
            herd.update()
        label = myfont.render(str(int(clock.get_fps())) + " fps", 2, (0, 0, 0))
        screen.blit(label, (WIDTH - 80, 2))
        collision = myfont.render(str(int(herd.collision_counter)) + " collisions", 2, (0, 0, 0))
        screen.blit(collision, (20, 2))
        pygame.display.update()
        msElapsed = clock.tick(FRAME_RATE)

if __name__ == '__main__':
    run()
