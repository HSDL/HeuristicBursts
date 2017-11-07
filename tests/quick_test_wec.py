# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual
import time


def test_simulation_of_wec():
    device = wec.wec.WEC()
    display = wec.wec_visual.wec_visual()

    device.add_body('sphere', 10, (400, 400.0), radius=25)

    display.display(device)

    device.add_rotational_body('sphere', 20, (450, 400.0), 0, 1, 0, 1000, 1000, radius=25)

    display.display(device)

    device.add_linear_body('sphere', 15, (300, 400.0), 0, 2, 100, 1000, 1000, radius=75)

    display.display(device)

    device.change_joint_type(0, 'rotational')

    display.display(device)

    device.remove_body_with_joint(2, 0, 'linear')

    display.display(device)

    device.remove_body_with_joint(1, 0, 'linear')

    display.display(device)

    device.remove_body(0)

    display.display(device)

    time.sleep(5)
