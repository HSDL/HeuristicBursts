import wec
import time as time
import matplotlib.pyplot as pp

# Instantiate a device and set simulation parameters
# wec.WEC.simulation_dt = 0.1
# wec.WEC.simulation_steps = 5000
# wec.WEC.error_bias = 0.02
# wec.WEC.forces.load_model('./data/full_network_structure.yml', './data/full_network_weights.h5')

# Create two bodies and a PTO
device = wec.WEC()
device.add_body('sphere', 651.2, (0, 0.0), radius=3)
device.add_body('sphere', 603.2, (10, 0), radius=3)
device.add_body('sphere', 550.2, (20, 0), radius=3)
device.add_body('sphere', 599.2, (30, 0), radius=3)
device.add_rotational_pto(0, 1, 0, 1000, 10000000)
device.add_rotational_pto(1, 2, 0, 1000, 10000000)
device.add_rotational_pto(2, 3, 0, 1000, 10000000)


for body in device.bodies:
    pp.plot(body['xyz'][:, 1])
    print(body['mass'])

t = time.time()
device.evaluate()
print(device.power)
print("Elapsed: ", time.time() - t)

for body in device.bodies:
    pp.plot(body['xyz'][:, 1])
    print(body['mass'])
pp.show()
