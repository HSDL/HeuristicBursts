# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('sphere', 650.0, (400, 400), radius=10)
    device.add_linear_body('sphere', 650.0, (430, 400), 0, 1000, 10000000, radius=10)
    device.add_rotational_body('sphere', 650.0, (370, 400), 0, 0, 1000, 10000000, radius=10)
    device.change_body_dimensions(0, radius=20)
    device.add_linear_body('sphere', 1000, (350, 350), 0, 1000, 10000000, radius=5)
    device.swap_bodies(1, 2)
    device.add_mooring_system((350, 150), 3, 1000, 10000000)
    device.add_linear_body('sphere', 1000, (400, 290), 1, 1000, 10000000, radius=15)
    device.add_mooring_system((250, 150), 2, 1000, 10000000)
    device.swap_bodies(3, 4)
    device.change_body_dimensions(1, radius=15)
    device.relocate_mooring_cable_attachment(1, 4)

    device.display_visual = True
    device.evaluate()
    print(device.power)
