# This test is for quickly testing functionality of individual rules
import wec.wec
import wec.wec_visual


def test_simulation_of_wec():
    device = wec.wec.WEC()

    device.add_body('cylinder', 650.0, (600, 0), radius=25, length=50, angle_offset=90)

    device.evaluate()
    print(device.bodies[0]["xyz"])
    print(device.power)
