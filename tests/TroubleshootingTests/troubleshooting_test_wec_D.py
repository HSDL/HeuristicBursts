# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('cylinder', 650.0, (400, 0), radius=5, length=10, angle_offset=0)
    device.add_linear_body('sphere', 650, (410, 0), 0, 1, 10, 1000, 10000000, radius=5)
    device.add_rotational_body('sphere', 650, (420, 0), 1, 2, 0, 1000, 10000000, radius=5)
    device.add_linear_body('cylinder', 650, (430, 0), 2, 3, 10, 1000, 10000000, radius=5, length=10, angle_offset=0)
    device.relocate_body_with_joint(3, 1, 'linear', (390, 0), 0)
    device.change_body_dimensions(3, radius=10, length=5)
    device.swap_bodies(3, 2)

    device.evaluate()
    print(device.power)
