import pygame

class wec_visual():

    def __init__(self):
        # initialize screen for displaying WEC design
        pygame.init()
        screen = pygame.display.set_mode((1000, 800))
        screen.fill((255, 255, 255))
        pygame.display.update()

    def display(self):
        pass

screen = wec_visual()
while True:
        pass