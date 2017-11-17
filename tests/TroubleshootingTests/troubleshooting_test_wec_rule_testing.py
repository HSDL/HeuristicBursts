# This test will eventually be used to show the example system created by series of low-tier operations
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('sphere', 650.0, (400, 400), radius=50)
    device.add_rotational_body('sphere', 650.0, (550, 400), 0, 0, 1000, 10000000, radius=50)
    device.add_rotational_body('sphere', 850.0, (550, 200), 1, 0, 1000, 10000000, radius=50)
    device.change_joint_type(1, 'rotational')
    device.add_linear_body('sphere', 650.0, (700, 0), 1, 1000, 10000000, radius=50)
    device.relocate_body_with_joint(3, 1, 'linear', (250, 400), 0)
    device.change_body_density(2, 1000.0)
    device.add_mooring_system((550, 100), 2, 1000, 10000000)
    device.relocate_mooring_fixed_body(0, (200, 100))
    device.add_linear_body('sphere', 650.0, (700, 400), 1, 1000, 10000000, radius=50)
    device.change_body_dimensions(0, radius=75)
    device.remove_body_with_joint(3, 1, 'linear')
    device.swap_bodies(0, 1)

    device.display_visual = True
    device.evaluate()
    print(device.power)
