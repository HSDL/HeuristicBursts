import pygame
import pymunk.pygame_util


class wec_visual():
    # Colors
    white = (255, 255, 255)
    black = (0, 0, 0)
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
        for n in range(0, int(self.SCREEN_WIDTH/50)+100):
            pygame.draw.line(self.screen, self.black, (n*50, 0), (n*50, self.SCREEN_HEIGHT))
        for n in range(0, int(self.SCREEN_HEIGHT/50)+100):
            pygame.draw.line(self.screen, self.black, (0, n*50), (self.SCREEN_WIDTH, n*50))
        # display_device = self.display_shift(device)
        device.world.debug_draw(options)
        pygame.display.update()

    def display_shift(self, device):
        display_device = device
        for index in range(0, len(display_device.bodies)):
            pass
            # display_device.world.shapes[index].position[1] += 400
            # print(display_device.world.bodies[index].position[1])

        return display_device