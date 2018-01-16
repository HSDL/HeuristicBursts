# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual
import time
import random
import pygame


def test_simulation_of_wec():
    device = wec.wec.WEC()
    display = wec.wec_visual.wec_visual()
    display.display(device)

    device.add_body('sphere', 650.0, (400, 400), radius=20)
    device.rule_perform(1, location=(0, 0))
    device.rule_perform(2, location=(0, 2))
    device.rule_perform(1, location=(1, 0))
    device.rule_perform(2, location=(2, 2))

    display.display(device)
    time.sleep(2)
    wait = True
    while wait:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    wait = False
    # device.add_body('sphere', 650.0, (449, 0), radius=25)
    # device.add_body('sphere', 650.0, (350, 0), radius=25)

    # device.add_rotational_body('sphere', 650.0, (451, 400), 0, 0, 1000, 10000000, radius=25)
    # device.add_rotational_body('sphere', 650.0, (502, 400), 1, 0, 1000, 10000000, radius=25)
    # device.rule_perform(3, removal=(2, 1, 'rotational'))

    device.increase_or_decrease_complexity(complexity_change_type='decrease',
                                           rule_length=2)
    display.display(device)
    time.sleep(2)
    wait = True
    while wait:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    wait = False

    for i in range(0, 5):
        device.change_design_scale(multiplier=0.25)
        display.display(device)
        for body in device.bodies:
            print('size: ', body['radius'])
        time.sleep(2)
        wait = True
        while wait:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:
                        wait = False

    num_ops = 5
    # for n in range(0, num_ops):
    #     op_select = random.randint(0, 2)
    #     if op_select == 0:
    #         device.increase_or_decrease_complexity()
    #     elif op_select == 1:
    #         device.change_design_scale()
    #     elif op_select == 2:
    #         device.replicate_pattern()
    #     display.display(device)
    #     time.sleep(2)
    #     wait = True
    #     while wait:
    #         for event in pygame.event.get():
    #             if event.type == pygame.KEYDOWN:
    #                 if event.key == pygame.K_RETURN:
    #                     wait = False

    # device.add_linear_body('sphere', 850.0, (450, 350), 1, 1000, 10000000, radius=25)
    # device.add_linear_body('sphere', 850.0, (400, 350), 0, 1000, 10000000, radius=25)
    # device.add_mooring_system((400, 50), 2, 10000000, 10000000)

    # device.create_initial_design(20)

    # for body in device.bodies
    # print(device.deletable_bodies):
    #     print(body['body'].position)

    # device.relocate_mooring_cable_attachment(0, 2)
    # device.rule_check()

    # print(' ')
    # for i in range (0, len(device.bodies)):
    #     print('Body: ', i, '; Position: ', device.bodies[i]['body'].position)
    # print(' ')
    # print('Validity: ', device.is_valid())

    # display = wec.wec_visual.wec_visual()
    # display.display(device)

    # device.replicate_pattern()
    # display.display(device)
    # time.sleep(2)
    # wait = True
    # while wait:
    #     for event in pygame.event.get():
    #         if event.type == pygame.KEYDOWN:
    #             if event.key == pygame.K_RETURN:
    #                 wait = False

    # device.display_visual = True
    # device.locate_center_of_gravity()

    # device.evaluate()

    # print(device.power)

