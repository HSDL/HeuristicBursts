import numpy as np
import pymunk as pm

from heuristic_bursts.abstract_base_solution import AbstractBaseSolution

import wec.spectrum as sp
import wec.excitation_forces as ef
import pkg_resources


class WEC(AbstractBaseSolution):
    simulation_dt = 0.1
    simulation_steps = 1000
    error_bias = 0.02
    spectrum = sp.Spectrum('bretschneider', fp=0.5, Hm0=2)
    forces = ef.ExcitationForces()
    forces.load_model(pkg_resources.resource_filename('wec', 'data/full_network_structure.yml'),
                      pkg_resources.resource_filename('wec', 'data/full_network_weights.h5'))
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

        # Start lists for PTO types
        self.linear_ptos = []
        self.rotary_ptos = []
        self.linear_ptos_data = []
        self.rotary_ptos_data = []


        # current iteration
        self.iter = 0

    def add_body(self, shape, density, position, **kwargs):

        # LUCAS: I added these to make sure that they have *some* value. Otherwise, the 'length' variable
        #  may be undefined when you try to use it on line 76
        radius = 0
        length = 0

        if shape is 'sphere':
            radius = kwargs['radius']
            volume = (4.0/3.0)*np.pi*np.power(radius, 3)
            mass = density * volume
            moment = 0.4*mass*np.power(radius, 2)
            coefficients = []
        elif shape is 'cylinder':
            radius = kwargs['radius']
            length = kwargs['length']
            volume = np.pi*np.power(radius,2)*length
            mass = density * volume
            moment = 0.25*mass*np.power(radius,2) + (1/12.)*mass*np.power(length,2)  # Moment about central diameter
            coefficients = []
        temp_body = pm.Body(mass=mass, moment=moment)
        temp_body.position = position
        shape = pm.Circle(temp_body, kwargs['radius'])
        self.world.add(temp_body)
        self.world.add(shape)
        self.bodies.append({"body": temp_body,
                            "coefficients": coefficients,
                            "density": density,
                            "volume": volume,
                            "mass": mass,
                            "moment": moment,
                            "radius": radius,
                            "length": length,
                            "xyz": np.zeros((self.simulation_steps, 2)),
                            "last_velocity": np.zeros(2),
                            "last_position": np.zeros(2),
                            "shape": shape})

    # LUCAS: Lower tier operations go here
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
        self.linear_ptos_data.append({"Linear PTO": temp_pto,
                                      "idxa": idxa,
                                      "idxb": idxb,
                                      "resting_length": resting_length,
                                      "stiffness": stiffness,
                                      "damping": damping})

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

    # LUCAS: Lower tier operations go here
    def add_rotational_pto(self, idxa, idxb, rest_angle, stiffness, damping):
        # Add damped spring
        a = self.bodies[idxa]["body"]
        b = self.bodies[idxb]["body"]
        temp_pto = pm.constraint.DampedRotarySpring(a, b, rest_angle, stiffness, damping)
        temp_pto.collide_bodies = False
        temp_pto.error_bias = self.error_bias
        self.world.add(temp_pto)
        self.rotary_ptos.append(temp_pto)
        self.rotary_ptos_data.append({"Rotational PTO": temp_pto,
                                      "idxa": idxa,
                                      "idxb": idxb,
                                      "rest_angle": rest_angle,
                                      "stiffness": stiffness,
                                      "damping": damping})

        # Add a pivot joint
        acg = a.local_to_world(a.center_of_gravity)
        bcg = b.local_to_world(b.center_of_gravity)
        temp_pivot = pm.constraint.PivotJoint(a, b, 0.5*(acg + bcg))
        temp_pivot.collide_bodies = False
        temp_pivot.error_bias = self.error_bias
        self.world.add(temp_pivot)

    def add_rotational_body(self, shape, density, position, idxa, idxb, rest_angle, stiffness, damping, **kwargs):
        self.add_body(shape, density, position, **kwargs)
        self.add_rotational_pto(idxa, idxb, rest_angle, stiffness, damping)

    def add_linear_body(self, shape, density, position, idxa, idxb, resting_length, stiffness, damping, **kwargs):
        self.add_body(shape, density, position, **kwargs)
        self.add_constrained_linear_pto(idxa, idxb, resting_length, stiffness, damping)

    # Note: all of these still need to update in World!
    def remove_body(self, index):
        del self.bodies[index]

    def remove_joint(self, index, joint_type):
        if joint_type is 'rotational':
            del self.rotary_ptos[index]
            del self.rotary_ptos_data[index]
        elif joint_type is 'linear':
            del self.linear_ptos[index]
            del self.linear_ptos_data[index]

    def remove_body_with_joint(self, body_index, joint_index, joint_type):
        self.remove_body(body_index)
        self.remove_joint(joint_index, joint_type)

    def change_joint_type(self, joint_initial_type, index):
        if joint_initial_type is 'rotational':
            resting_length = 1
            self.add_constrained_linear_pto(self.rotary_ptos_data[index]["idxa"],
                                            self.rotary_ptos_data[index]["idxb"],
                                            resting_length,
                                            self.rotary_ptos_data[index]["stiffness"],
                                            self.rotary_ptos_data[index]["damping"])
        elif joint_initial_type is 'linear':
            rest_angle = 0
            self.add_rotational_pto(self.linear_ptos_data[index]["idxa"],
                                    self.linear_ptos_data[index]["idxb"],
                                    rest_angle,
                                    self.linear_ptos_data[index]["stiffness"],
                                    self.linear_ptos_data[index]["damping"])

        self.remove_joint(index, joint_initial_type)

    def change_body_dimensions(self, index, **kwargs):
        temp_body = self.bodies[index]

        if temp_body["shape"] is 'sphere':
            radius = kwargs['radius']
            length = temp_body["length"]
            volume = (4.0 / 3.0) * np.pi * np.power(radius, 3)
            mass = temp_body["density"] * volume
            moment = 0.4*mass*np.power(radius, 2)
        elif temp_body["shape"] is 'cylinder':
            radius = kwargs['radius']
            length = kwargs['length']
            volume = np.pi*np.power(radius,2)*length
            mass = temp_body["density"] * volume
            moment = 0.25 * mass * np.power(radius, 2) + (1 / 12.) * mass * np.power(length, 2)

        self.bodies[index]["radius"] = radius
        self.bodies[index]["length"] = length
        self.bodies[index]["volume"] = volume
        self.bodies[index]["mass"] = mass
        self.bodies[index]["moment"] = moment

    def change_body_density(self, index, density):
        temp_body = self.bodies[index]
        mass = density * temp_body["volume"]

        if temp_body["shape"] is 'sphere':
            moment = 0.4*mass*np.power(temp_body["radius"], 2)
        elif temp_body["shape"] is 'cylinder':
            moment = 0.25*mass*np.power(temp_body["radius"], 2) + (1/12.)*mass*np.power(temp_body["length"], 2)

        self.bodies[index]["density"] = density
        self.bodies[index]["mass"] = mass
        self.bodies[index]["moment"] = moment

    # Incomplete function; need to figure out best way to go about this
    def relocate_body_with_joint(self, body_index, joint_index, joint_type):
        temp_body = self.bodies[body_index]
        self.remove_body_with_joint(body_index, joint_index, joint_type)

    def swap_bodies(self):
        pass

    def add_mooring_system(self):
        pass

    def remove_mooring_system(self):
        pass

    def relocate_mooring_cable_attachment(self):
        pass

    def relocate_mooring_fixed_body(self):
        pass

    # LUCAS: Put lower tier operations above here

    # LUCAS: Higher tier operations will go here eventually too. These should follow the function definitions as defined
    #        in the abstract base solution class.

    ### Everything below here is for evaluation

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

    def __deepcopy__(self):
        asdf = 1
