# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual
import time


def test_simulation_of_wec():
    device = wec.wec.WEC()
    display = wec.wec_visual.wec_visual()
    display.display(device)

    device.add_body('sphere', 650.0, (400, 400), radius=25)
    # device.add_body('sphere', 650.0, (449, 0), radius=25)
    # device.add_body('sphere', 650.0, (350, 0), radius=25)

    device.add_rotational_body('sphere', 650.0, (450, 400), 0, 0, 1000, 10000000, radius=25)
    device.add_rotational_body('sphere', 650.0, (500, 400), 1, 0, 1000, 10000000, radius=25)
    # device.add_linear_body('sphere', 850.0, (450, 350), 1, 1000, 10000000, radius=25)
    # device.add_linear_body('sphere', 850.0, (400, 380), 0, 1000, 10000000, radius=5)
    # device.add_mooring_system((400, 50), 0, 10000000, 10000000)

    # device.change_body_dimensions(1, radius=50)
    device.create_initial_design(20)
    # for body in device.bodies:
    #     print(body['body'].position)

    # device.relocate_mooring_cable_attachment(0, 2)

    # device.swap_bodies(1, 3)
    # device.swap_bodies(1, 2)
    # device.change_body_dimensions(0, radius=45)
    # device.change_body_dimensions(0, radius=50)
    # device.change_body_density(0, 200)
    # device.change_body_density(0, 650.0)
    # device.relocate_body_with_joint(2, 1, 'rotational', (300, 0), 0)
    # device.relocate_body_with_joint(2, 1, 'rotational', (600, 0), 1)
    # print(' ')
    # for i in range (0, len(device.bodies)):
    #     print('Body: ', i, '; Position: ', device.bodies[i]['body'].position)
    # print(' ')
    # print('Validity: ', device.is_valid())
    device.display_visual = True
    device.evaluate()
    print(device.power)

