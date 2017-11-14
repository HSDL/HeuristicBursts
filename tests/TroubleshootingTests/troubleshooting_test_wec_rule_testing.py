# This test will eventually be used to show the example system created by series of low-tier operations
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('sphere', 650.0, (400, 100), radius=50)
    device.add_rotational_body('sphere', 650.0, (500, 100), 0, 1, 0, 1000, 10000000, radius=50)
    device.add_rotational_body('sphere', 800.0, (500, 0), 1, 2, 0, 1000, 10000000, radius=50)
    device.change_joint_type(1, 'rotational')
    device.add_linear_body('sphere', 650.0, (600, 100), 1, 3, 100, 1000, 10000000, radius=50)
    device.relocate_body_with_joint(3, 1, 'linear', (300, 100), 0)
    device.change_body_density(2, 850.0)
    device.add_mooring_system((400, 0), 0, 100, 1000, 10000000)
    device.relocate_mooring_fixed_body(0, (200, 0))
    device.add_linear_body('sphere', 650.0, (600, 100), 1, 3, 100, 1000, 10000000, radius=50)
    device.change_body_dimensions(0, radius=25)
    device.remove_body_with_joint(3, 1, 'linear')
    device.swap_bodies(0, 1)

    device.evaluate()

    print(device.power)
