# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('cylinder', 500.0, (400, 400), radius=50, length=100, angle_offset=0)
    device.add_linear_body('sphere', 650, (500, 400), 0, 1000, 10000000, radius=50)
    device.add_rotational_body('sphere', 650, (600, 400), 1, 0, 1000, 10000000, radius=50)
    device.add_linear_body('cylinder', 500, (700, 400), 2, 1000, 10000000, radius=50, length=100, angle_offset=0)
    device.relocate_body_with_joint(3, 1, 'linear', (300, 400), 0)
    device.change_body_dimensions(3, radius=50, length=50, angle_offset=0)
    device.swap_bodies(3, 1)

    device.display_visual = True
    device.evaluate()
    print(device.power)
