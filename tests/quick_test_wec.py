# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual
import time


def test_simulation_of_wec():
    device = wec.wec.WEC()
    # display = wec.wec_visual.wec_visual()
    # display.display(device)

    device.add_body('sphere', 650.0, (400, 0), radius=50)
    # device.add_rotational_body('sphere', 825.0, (400, -100), 0, 1, 0, 1000, 10000000, radius=50)
    # # device.add_rotational_body('sphere', 650.0, (600, 0), 1, 2, 0, 1000, 10000000, radius=50)
    #
    # device.add_mooring_system((400, -250), 1, 150, 10000000, 10000000)
    # device.relocate_mooring_cable_attachment(0, 2)

    # device.swap_bodies(1, 2)
    # device.swap_bodies(1, 2)
    # device.change_body_dimensions(0, radius=45)
    # device.change_body_dimensions(0, radius=50)
    # device.change_body_density(0, 200)
    # device.change_body_density(0, 650.0)
    # device.relocate_body_with_joint(2, 1, 'rotational', (300, 0), 0)
    # device.relocate_body_with_joint(2, 1, 'rotational', (600, 0), 1)

    device.evaluate()
    print(device.power)

    # time.sleep(5)
