# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual
import time


def test_simulation_of_wec():
    device = wec.wec.WEC()
    display = wec.wec_visual.wec_visual()
    display.display(device)

    device.add_body('sphere', 650.0, (400, 400), radius=25)
    # device.add_rotational_body('sphere', 650, (450, 400), 0, 0, 1000, 10000000, radius=25)
    # device.add_rotational_body('sphere', 650, (500, 400), 1, 0, 1000, 10000000, radius=25)
    # device.add_rotational_body('sphere', 650, (550, 400), 2, 0, 1000, 10000000, radius=25)
    # device.add_rotational_body('sphere', 650, (600, 400), 3, 0, 1000, 10000000, radius=25)

    device.create_initial_design(20)
    # device.rule_perform(3)
    # device.rule_perform(2)
    # device.rule_perform(1)
    # device.rule_perform(2)

    device.display_visual = True
    device.evaluate()
    print(device.power)
