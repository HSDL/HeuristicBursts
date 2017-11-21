# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('sphere', 650.0, (400, 400), radius=10)
    device.add_linear_body('sphere', 650.0, (450, 400), 0, 1000, 10000000, radius=10)
    device.add_rotational_body('sphere', 650.0, (350, 400), 0, 0, 1000, 10000000, radius=10)
    device.change_body_density(1, 400.0)
    device.remove_body_with_joint(1, 0, 'linear')
    device.swap_bodies(0, 1)
    device.add_mooring_system((300, 150), 0, 1000, 10000000)
    device.change_body_dimensions(0, radius=15)
    device.add_linear_body('sphere', 1100, (400, 350), 0, 1000, 10000000, radius=10)
    device.change_body_dimensions(1, radius=15)
    device.swap_bodies(0, 1)
    device.remove_body_with_joint(1, 0, 'rotational')
    device.add_rotational_body('sphere', 500, (450, 400), 0, 0, 1000, 10000000, radius=15)

    device.display_visual = True
    device.evaluate()
    print(device.power)
