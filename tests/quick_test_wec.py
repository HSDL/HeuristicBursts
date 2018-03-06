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


    for i in range(0, len(device.bodies)):
        print('Body: ', i, '; Position: ', device.bodies[i]['body'].position, 'Radius:',
              device.bodies[i]['radius'])
    print(' ')
    print(' ')

    display.display(device)
    time.sleep(2)
    display.wait_to_continue()

    # device.hightier_rule_perform(4)
    # device.lowtier_rule_perform(device.lowtier_rule_select())
    # device.hightier_rule_perform(5)
    # device.lowtier_rule_perform(6)
    # device.hightier_rule_perform(device.hightier_rule_select())


    # for i in range(0, len(device.bodies)):
    #     print('Body: ', i, '; Position: ', device.bodies[i]['body'].position, 'Radius:', device.bodies[i]['radius'])
    # print(' ')
    # print(' ')
    #
    # device.change_design_scale()
    # display.display(device)
    # time.sleep(2)
    # display.wait_to_continue()
    # device.rule_check()
    # print(device.addition_locations)
    # print('')
    #
    # for i in range(0, len(device.bodies)):
    #     print('Body: ', i, '; Position: ', device.bodies[i]['body'].position, 'Radius:',
    #           device.bodies[i]['radius'])
    # print(' ')
    # print(' ')
    # #
    # for i in range(0, 5):
    #     device.increase_or_decrease_complexity()
    #     display.display(device)
    #     time.sleep(2)
    #     wait = True
    #     while wait:
    #         for event in pygame.event.get():
    #             if event.type == pygame.KEYDOWN:
    #                 if event.key == pygame.K_RETURN:
    #                     wait = False

    # num_ops = 10
    # for n in range(0, num_ops):
    #     rule = device.lowtier_rule_select()
    #     device.lowtier_rule_perform(rule)
    #     display.display(device)
    #     time.sleep(2)
    #     display.wait_to_continue()

    # device.create_initial_design(20)

    # device.replicate_pattern()
    # display.display(device)
    # time.sleep(2)
    # wait = True
    # while wait:
    #     for event in pygame.event.get():
    #         if event.type == pygame.KEYDOWN:
    #             if event.key == pygame.K_RETURN:
    #                 wait = False

    print(device.applied_rules)

    # device.display_visual = True
    # device.locate_center_of_gravity()

    device.evaluate()

    print(device.power)

