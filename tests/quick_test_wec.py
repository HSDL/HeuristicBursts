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
    # device.lowtier_rule_perform(1, location=(4, 0), radius=1)
    # device.lowtier_rule_perform(1, location=(3, 2), radius=1)
    # device.lowtier_rule_perform(2, location=(3, 3), radius=1)
    # device.hightier_rule_perform(2)
    # device.hightier_rule_perform(4, multiplier=0.5)
    # device.hightier_rule_perform(6)
    # device.hightier_rule_perform(2)
    # device.hightier_rule_perform(3)
    # device.hightier_rule_perform(6)

    # display.display(device)
    # time.sleep(2)
    # display.wait_to_continue()
    # device.lowtier_rule_perform(7, body_a=0, body_b=3)
    # display.display(device)
    # time.sleep(2)
    # display.wait_to_continue()
    # device.lowtier_rule_perform(7, body_a=0, body_b=1)


    # device.hightier_rule_perform(device.hightier_rule_select())


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

    display.display(device)
    time.sleep(2)
    display.wait_to_continue()

    print(device.applied_rules)

    device.display_visual = True

    device.evaluate()
    print(device.power)
    print(device.mass)
    print(device.power/device.mass)

