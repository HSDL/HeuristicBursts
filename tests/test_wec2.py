# This test will eventually be used to show the example system created by series of low-tier operations
import wec.wec


def test_simulation_of_wec():
    device = wec.wec.WEC()
    device.add_body('cylinder', 651.2, (0, 0.0), radius=4, length=5)
    device.add_body('sphere', 603.2, (10, 0), radius=3, length =3)
    device.add_body('cylinder', 550.2, (20, 0), radius=3, length=5)
    device.add_body('cylinder', 599.2, (30, 0), radius=3, length=3)
    device.add_rotational_pto(0, 1, 0, 1000, 10000000)
    device.add_constrained_linear_pto(1, 2, 2, 1000, 10000000)
    device.add_rotational_pto(2, 3, 0, 1000, 10000000)
    device.remove_body(1)
    for body in device.bodies:
        print(body['mass'])

    device.evaluate()
