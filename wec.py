import pymunk as pm
import numpy as np
from utils import excitation_forces as ef
from utils import spectrum as sp
from abstract_base_solution import AbstractBaseSolution


class WEC(AbstractBaseSolution):
    simulation_dt = 0.1
    simulation_steps = 1000
    error_bias = 0.02
    spectrum = sp.Spectrum('bretschneider', fp=0.5, Hm0=2)
    forces = ef.ExcitationForces()
    forces.load_model('./data/full_network_structure.yml', './data/full_network_weights.h5')
    rho_w = 1000
    gravity = -9.81
    number_of_metrics = 3

    def __init__(self):
        #
        self.world = pm.Space()
        self.world.gravity = (0, -9.81)
        self.world.damping = 0.95

        # Save position history
        self.bodies = []

        # Initialize quality
        self.mass = 0
        self.power = 0
        self.pieces = 0

        # Load neural network for simulations
        self.linear_ptos = []
        self.rotary_ptos = []

        # current iteration
        self.iter = 0

    def add_body(self, shape, density, position, **kwargs):
        if shape is 'sphere':
            radius = kwargs['radius']
            volume = (4.0/3.0)*np.pi*np.power(radius, 3)
            mass = density * volume
            moment = 0.4*mass*np.power(radius, 2)
            # Get coefficients by first getting points on body
            # Next build voxel map for points on body
            # Finally
            coefficients = []
        temp_body = pm.Body(mass=mass, moment=moment)
        temp_body.position = position
        self.world.add(temp_body)
        self.bodies.append({"body": temp_body,
                            "coefficients": coefficients,
                            "density": density,
                            "volume": volume,
                            "mass": mass,
                            "moment": moment,
                            "radius": radius,
                            "xyz": np.zeros((self.simulation_steps, 2)),
                            "last_velocity": np.zeros(2),
                            "last_position": np.zeros(2)})

    def add_constrained_linear_pto(self, idxa, idxb, resting_length, stiffness, damping):
        # Add damped spring
        a = self.bodies[idxa]["body"]
        b = self.bodies[idxb]["body"]
        temp_pto = pm.constraint.DampedSpring(a, b, a.center_of_gravity, b.center_of_gravity,
                                              resting_length, stiffness, damping)
        temp_pto.collide_bodies = False
        temp_pto.error_bias = self.error_bias
        self.world.add(temp_pto)
        self.linear_ptos.append(temp_pto)

        # Add groove to constrain linear motion
        temp_groove = pm.constraint.GrooveJoint(a, b, a.center_of_gravity,
                                                a.center_of_gravity + 2*(b.position - a.position), b.center_of_gravity)
        temp_groove.collide_bodies = False
        temp_groove.error_bias = self.error_bias
        self.world.add(temp_groove)
        temp_groove = pm.constraint.GrooveJoint(b, a, b.center_of_gravity,
                                                b.center_of_gravity + 2*(a.position - b.position), a.center_of_gravity)
        temp_groove.collide_bodies = False
        temp_groove.error_bias = self.error_bias

        self.world.add(temp_groove)

    def add_rotational_pto(self, idxa, idxb, rest_angle, stiffness, damping):
        # Add damped spring
        a = self.bodies[idxa]["body"]
        b = self.bodies[idxb]["body"]
        temp_pto = pm.constraint.DampedRotarySpring(a, b, rest_angle, stiffness, damping)
        temp_pto.collide_bodies = False
        temp_pto.error_bias = self.error_bias
        self.world.add(temp_pto)
        self.rotary_ptos.append(temp_pto)

        # Add a pivot joint
        acg = a.local_to_world(a.center_of_gravity)
        bcg = b.local_to_world(b.center_of_gravity)
        temp_pivot = pm.constraint.PivotJoint(a, b, 0.5*(acg + bcg))
        temp_pivot.collide_bodies = False
        temp_pivot.error_bias = self.error_bias
        self.world.add(temp_pivot)


    def add_buoyant_force(self):
        for body in self.bodies:
            r = body["radius"]
            if r + body["body"].position[1] < 0:
                displaced_mass = body["volume"]*self.rho_w
            elif body["body"].position[1] - r > 0:
                displaced_mass = 0
            else:
                h = r - body["body"].position[1]
                c = np.sqrt(h*(2*r-h))
                displaced_mass = self.rho_w*(np.pi/6)*h*(3*c*c+h*h)

            faddl = -displaced_mass*self.world.gravity
            body["body"].apply_force_at_local_point(faddl, body["body"].center_of_gravity)

    def add_excitation_force(self):
        for body in self.bodies:
            if body['body'].position[1] > -10:
                body["body"].apply_force_at_local_point((0, body['mass']*np.sin(self.iter/100 + body['body'].position[0])), body["body"].center_of_gravity)

    def add_radiative_force(self):
        for body in self.bodies:
            body["body"].apply_force_at_local_point((0, 0), body["body"].center_of_gravity)

    def add_viscous_force(self):
        for body in self.bodies:
            # Compute viscous drag of sphere perpendicular to force,
            Cd = 0.47
            v = body["body"].velocity
            A = np.pi*np.power(body["radius"], 2)
            f = -0.5*self.rho_w*Cd*A*v*np.linalg.norm(v)

            body["body"].apply_force_at_local_point(f, body["body"].center_of_gravity)

    def pull_position_data(self):
        for i in range(len(self.bodies)):
            self.bodies[i]['xyz'][self.iter, 0] = self.bodies[i]['body'].position[0]
            self.bodies[i]['xyz'][self.iter, 1] = self.bodies[i]['body'].position[1]

    def evaluate(self):
        energy = np.zeros(2)
        while self.iter < self.simulation_steps:

            # Add forces
            self.add_buoyant_force()
            self.add_excitation_force()
            # self.add_radiative_force()
            self.add_viscous_force()

            # Step the simulation forward
            self.world.step(self.simulation_dt)

            # Perform corrections
            for body in self.bodies:
                body['body'].velocity = 0.5*(body['body'].velocity + body['last_velocity'])
                dxy = body['body'].position - body['last_position']
                dv = np.linalg.norm(body['body'].velocity)/np.linalg.norm(body['last_velocity'])
                if np.isinf(dv) or np.isnan(dv):
                    dv = 1.0
                body['body'].position = body['last_position'] + dv*dxy

                body['last_velocity'] = body['body'].velocity
                body['last_position'] = body['body'].position
                # print(dv, dxy, body['last_position'], body['last_velocity'])

            # Pull position data
            self.pull_position_data()

            # Track PTO energy extraction
            for pto in self.linear_ptos:
                relative_velocity = np.linalg.norm(pto.a.velocity - pto.b.velocity)
                energy[0] += np.power(relative_velocity, 2)*pto.damping*self.simulation_dt
            for pto in self.rotary_ptos:
                relative_velocity = np.abs(pto.a.angular_velocity - pto.b.angular_velocity)
                energy[1] += np.power(relative_velocity, 2)*pto.damping*self.simulation_dt

            self.iter += 1
        self.iter = 0

        # Calculate power
        self.power = energy/(self.simulation_dt*self.simulation_steps)

        # Calculate mass
        self.mass = 0
        for body in self.bodies:
            self.mass += body['mass']

        # Calculate number of pieces
        self.pieces = len(self.linear_ptos) + len(self.rotary_ptos) + len(self.bodies)

        return [self.mass, -self.power, self.pieces]
