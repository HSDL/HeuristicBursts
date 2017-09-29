import wec.wec as w

def test_simulation_of_wec():
    # Create two bodies and a PTO
    device = w.WEC()
    device.add_body('sphere', 651.2, (0, 0.0), radius=3)
    device.add_body('sphere', 603.2, (10, 0), radius=3)
    device.add_body('sphere', 550.2, (20, 0), radius=3)
    device.add_body('sphere', 599.2, (30, 0), radius=3)
    device.add_rotational_pto(0, 1, 0, 1000, 10000000)
    device.add_rotational_pto(1, 2, 0, 1000, 10000000)
    device.add_rotational_pto(2, 3, 0, 1000, 10000000)

    for body in device.bodies:
        print(body['mass'])

    device.evaluate()