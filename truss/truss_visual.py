import pygame


class truss_visual():
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

    # Scale multiplier
    scale_multi = 25

    def __init__(self):
        # initialize screen for displaying WEC design
        pygame.init()
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        self.screen.fill(self.white)
        pygame.display.update()

    def display(self, design):
        self.screen.fill((255, 255, 255))

        # for n in range(0, int(self.SCREEN_WIDTH/50)+100):
        #     pygame.draw.line(self.screen, self.black, (n*50, 0), (n*50, self.SCREEN_HEIGHT))
        # for n in range(0, int(self.SCREEN_HEIGHT/50)+100):
        #     pygame.draw.line(self.screen, self.black, (0, n*50), (self.SCREEN_WIDTH, n*50))

        for index in range(len(design.con)):
            member = design.con[index]
            size = design.sizes[index]

            coordinates_a = design.coord[member[0]]
            coordinates_b = design.coord[member[1]]

            x_a = coordinates_a[0]*self.scale_multi + 400
            y_a = -(coordinates_a[1]*self.scale_multi - 400)

            x_b = coordinates_b[0]*self.scale_multi + 400
            y_b = -(coordinates_b[1]*self.scale_multi - 400)

            pygame.draw.line(self.screen, self.orange, (x_a, y_a), (x_b, y_b), int(size))

        for index in range(len(design.coord)):
            joint = design.coord[index]

            coordinates = joint

            x = int(coordinates[0]*self.scale_multi + 400)
            y = int(-(coordinates[1]*self.scale_multi - 400))

            color = self.black
            # if joint['joint_type'] == 'fixed':
            #     color = self.black
            # elif joint['joint_type'] == 'free':
            #     color = self.grey
            pygame.draw.circle(self.screen, color, (x, y), 10)

            if index in design.fixed_joints:
                pygame.draw.line(self.screen, self.grey, (x-15, y+10), (x+15, y+10), 10)

        pygame.display.update()

    def wait_to_continue(self):
        pygame.draw.circle(self.screen, self.black, (100, 100), 52)
        pygame.draw.circle(self.screen, self.green, (100, 100), 50)

        wait = True
        while wait:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:
                        wait = False

        pygame.draw.circle(self.screen, self.black, (100, 100), 52)
        pygame.draw.circle(self.screen, self.red, (100, 100), 50)

        pygame.display.update()