# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual
import time


def test_simulation_of_wec():
    device = wec.wec.WEC()
    # display = wec.wec_visual.wec_visual()
    # display.display(device)

    device.add_body('sphere', 650.0, (400, 400), radius=25)
    # device.add_body('sphere', 650.0, (449, 0), radius=25)
    # device.add_body('sphere', 650.0, (350, 0), radius=25)
    # device.rule_perform(1)

    # device.add_rotational_body('sphere', 650.0, (451, 400), 0, 0, 1000, 10000000, radius=25)
    # device.add_rotational_body('sphere', 650.0, (500, 400), 1, 0, 1000, 10000000, radius=25)
    # device.add_linear_body('sphere', 850.0, (450, 350), 1, 1000, 10000000, radius=25)
    # device.add_linear_body('sphere', 850.0, (400, 350), 0, 1000, 10000000, radius=25)
    # device.add_mooring_system((400, 50), 2, 10000000, 10000000)

    device.create_initial_design(20)

    # for body in device.bodies:
    #     print(body['body'].position)

    # device.relocate_mooring_cable_attachment(0, 2)
    device.rule_check()
    print(device.addition_locations)

    # print(' ')
    # for i in range (0, len(device.bodies)):
    #     print('Body: ', i, '; Position: ', device.bodies[i]['body'].position)
    # print(' ')
    # print('Validity: ', device.is_valid())

    display = wec.wec_visual.wec_visual()
    display.display(device)
    # device.display_visual = True
    device.evaluate()
    print(device.power)

