# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('cylinder', 650.0, (600, 100), radius=25, length=50, angle_offset=180)

    device.evaluate()
    print(device.power)
