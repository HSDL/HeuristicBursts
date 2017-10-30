# This test is for quickly testing functionality of individual rules
import wec.wec


def test_simulation_of_wec():
    device = wec.wec.WEC()
    device.add_body('cylinder', 10, (0, 0.0), radius=1, length=2)

    print(device.bodies[0])

    device.change_body_dimensions(0, radius=2, length=4)

    print(device.bodies[0])

    device.evaluate()
