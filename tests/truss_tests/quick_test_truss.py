import truss.truss
import truss.truss_visual

import time
import random


def test_simulation_of_truss():
    design = truss.truss.Truss()
    display = truss.truss_visual.truss_visual()
    display.display(design)
    time.sleep(1)
    display.wait_to_continue()

    design.hightier_rule_perform(3, scale_multiplier=0.25, scale_type='joint position')
    design.rule_check()
    print('')
    print('Rule: 1')
    print('Deletable:', design.deletable_joints)
    print('Joinable:', design.joinable_sets)
    print('Rediagonable:', design.re_diagonable_members)
    print('Movable:', design.moveable_joints)
    print('Validity:', design.is_valid())
    print('')

    display.display(design)
    time.sleep(1)
    display.wait_to_continue()

    for num in range(0, 20):
        rule = design.lowtier_rule_select()
        design.lowtier_rule_perform(rule)
        design.rule_check()
        print('')
        print('Rule:', rule)
        print('Deletable:', design.deletable_joints)
        print('Joinable:', design.joinable_sets)
        print('Rediagonable:', design.re_diagonable_members)
        print('Movable:', design.moveable_joints)
        print('Validity:', design.is_valid())
        print('')
        display.display(design)
        time.sleep(1)
        display.wait_to_continue()

    # display.display(design)
    # time.sleep(1)
    # display.wait_to_continue()
    print('All applied rules:', design.applied_rules)

test_simulation_of_truss()
