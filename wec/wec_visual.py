import pygame
import pymunk.pygame_util


class wec_visual():
    # Colors
    white = (255, 255, 255)
    light_blue = (0, 128, 255)

    # Screen Dimensions
    SCREEN_WIDTH = 1000
    SCREEN_HEIGHT = 800

    def __init__(self):
        # initialize screen for displaying WEC design
        pygame.init()
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        self.screen.fill(self.white)
        pygame.display.update()

    def display(self, device):

        options = pymunk.pygame_util.DrawOptions(self.screen)
        self.screen.fill((255, 255, 255))
        pygame.draw.rect(self.screen, self.light_blue, (0, self.SCREEN_HEIGHT/2, self.SCREEN_WIDTH, self.SCREEN_HEIGHT/2))
        device.world.debug_draw(options)
        pygame.display.update()

    def display_shift(self, device):
        pass