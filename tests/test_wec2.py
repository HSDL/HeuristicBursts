# This test will eventually be used to show the example system created by series of low-tier operations
import wec.wec
import wec.wec_visual
import time


def test_simulation_of_wec():
    device = wec.wec.WEC()
    # display = wec.wec_visual.wec_visual()

    # device.add_body('sphere', 651.2, (0, 0.0), radius=3)
    # device.add_rotational_body('sphere', 603.2, (10, 0), 0, 1, 0, 1000, 10000000, radius=3)
    # device.add_rotational_body('sphere', 550.2, (20, 0), 1, 2, 0, 1000, 10000000, radius=3)
    # device.add_rotational_body('sphere', 599.2, (30, 0), 2, 3, 0, 1000, 10000000, radius=3)

    device.add_body('sphere', 650.0, (400, 0), radius=10)
    device.add_rotational_body('sphere', 650.0, (430, 0), 0, 0, 1000, 10000000, radius=10)
    device.add_rotational_body('sphere', 800.0, (430, -30), 1, 0, 1000, 10000000, radius=10)
    device.change_joint_type(1, 'rotational')
    device.add_linear_body('sphere', 650.0, (460, 0), 1, 1000, 10000000, radius=10)
    device.relocate_body_with_joint(3, 1, 'linear', (370, 0), 0)
    device.change_body_density(2, 850.0)
    device.add_mooring_system((400, -150), 0, 1000, 10000000)
    # device.relocate_mooring_fixed_body(0, (200, -150))
    # device.add_linear_body('sphere', 650.0, (600, 0), 1, 3, 10, 1000, 10000000, radius=50, length=100)
    # device.change_body_dimensions(0, radius=50, length=50)
    # device.remove_body_with_joint(3, 1, 'linear')
    # device.swap_bodies(0, 1)

    device.evaluate()

    print(device.power)
