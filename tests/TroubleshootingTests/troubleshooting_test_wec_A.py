# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('sphere', 650.0, (400, 0), radius=5)
    device.add_rotational_body('sphere', 650.0, (410, 0), 0, 1, 0, 1000, 10000000, radius=5)
    device.add_rotational_body('sphere', 650.0, (420, 0), 1, 2, 0, 1000, 10000000, radius=5)
    # device.add_mooring_system((400, -100), 0, 100, 1000, 10000000)

    device.evaluate()
    print(device.power)
