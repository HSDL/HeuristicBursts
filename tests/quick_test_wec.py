# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual
import time


def test_simulation_of_wec():
    device = wec.wec.WEC()
    device.add_body('sphere', 10, (400, 400.0), radius=25)
    device.add_linear_body('sphere', 20, (500, 400.0), 0, 1, 50, 10000, 10000, radius=25)
    device.add_rotational_body('sphere', 10, (300, 400.0), 2, 0, 0, 20000, 40000, radius=25)
    device.add_linear_body('sphere', 20, (400, 300.0), 3, 0, 50, 15000, 20000, radius=25)

    display = wec.wec_visual.wec_visual()
    display.display(device)

    time.sleep(5)