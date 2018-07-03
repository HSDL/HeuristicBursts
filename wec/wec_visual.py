import pygame
import pymunk.pygame_util


class wec_visual():
    # Colors
    white = (255, 255, 255)
    grey = (100, 100, 100)
    black = (0, 0, 0)
    light_blue = (0, 128, 255)
    red = (175, 0, 0)
    green = (0, 175, 0)
    orange = (255, 69, 0)
    light_yellow = (238, 221, 130)
    olive = (107, 142, 35)
    dark_orange = (255, 69, 0)
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
        pygame.event.get()
        device.world.debug_draw(options)
        for body in device.bodies:
            radius = body['radius']
            position = body['body'].position
            position[0] = int(position[0]*10 + 400)
            position[1] = -int(position[1]*10 - 400)
            pygame.draw.circle(self.screen, self.black, position, int(radius * 10) + 1)
            pygame.draw.circle(self.screen, self.light_yellow, position, int(radius*10))
        for rotational_pto in device.rotary_ptos_data:
            idxa = rotational_pto['idxa']
            idxb = rotational_pto['idxb']
            pos_a = device.bodies[idxa]['body'].position
            pos_a[0] = int(pos_a[0]*10 + 400)
            pos_a[1] = -int(pos_a[1]*10 - 400)
            pos_b = device.bodies[idxb]['body'].position
            pos_b[0] = int(pos_b[0] * 10 + 400)
            pos_b[1] = -int(pos_b[1] * 10 - 400)
            pygame.draw.line(self.screen, self.olive, pos_a, pos_b, 3)
            pygame.draw.circle(self.screen, self.black, pos_a, 3)
            pygame.draw.circle(self.screen, self.black, pos_b, 3)
            pygame.draw.circle(self.screen, self.black,
                               (int((pos_a[0]+pos_b[0])/2), int((pos_a[1]+pos_b[1])/2)),
                               device.pto_size*4)
            pygame.draw.circle(self.screen, self.olive,
                               (int((pos_a[0] + pos_b[0]) / 2), int((pos_a[1] + pos_b[1]) / 2)),
                               device.pto_size * 4 - 2)
        for linear_pto in device.linear_ptos_data:
            idxa = linear_pto['idxa']
            idxb = linear_pto['idxb']
            pos_a = device.bodies[idxa]['body'].position
            pos_a[0] = int(pos_a[0] * 10 + 400)
            pos_a[1] = -int(pos_a[1] * 10 - 400)
            pos_b = device.bodies[idxb]['body'].position
            pos_b[0] = int(pos_b[0] * 10 + 400)
            pos_b[1] = -int(pos_b[1] * 10 - 400)
            pygame.draw.line(self.screen, self.dark_orange, pos_a, pos_b, 3)
            pygame.draw.circle(self.screen, self.black, pos_a, 3)
            pygame.draw.circle(self.screen, self.black, pos_b, 3)
            pygame.draw.rect(self.screen, self.black,
                             (int((pos_a[0] + pos_b[0]) / 2) - device.pto_size*3,
                              int((pos_a[1] + pos_b[1]) / 2)- device.pto_size*3,
                              device.pto_size * 6, device.pto_size * 6))
            pygame.draw.rect(self.screen, self.dark_orange,
                             (int((pos_a[0] + pos_b[0]) / 2) - device.pto_size * 3 + 2,
                              int((pos_a[1] + pos_b[1]) / 2) - device.pto_size * 3 + 2,
                              device.pto_size * 6 - 4, device.pto_size * 6 - 4))

        pygame.draw.circle(self.screen, self.black, (100, 100), 52)
        pygame.draw.circle(self.screen, self.red, (100, 100), 50)

        pygame.display.update()

    def wait_to_continue(self):
        pygame.draw.circle(self.screen, self.black, (100, 100), 52)
        pygame.draw.circle(self.screen, self.green, (100, 100), 50)
        pygame.display.update()
        wait = True
        while wait:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:
                        wait = False
        pygame.draw.circle(self.screen, self.black, (100, 100), 52)
        pygame.draw.circle(self.screen, self.red, (100, 100), 50)

        pygame.display.update()

    def display_shift(self, device):
        display_device = device
        for index in range(0, len(display_device.bodies)):
            pass
            # display_device.world.shapes[index].position[1] += 400
            # print(display_device.world.bodies[index].position[1])

        return display_device