# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('cylinder', 500.0, (400, 400), radius=25, length=100, angle_offset=45)

    device.display_visual = True
    device.evaluate()
    print(device.power)
