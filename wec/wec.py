import numpy as np
import pymunk as pm

from heuristic_bursts.abstract_base_solution import AbstractBaseSolution

import wec.spectrum as sp
import wec.excitation_forces as ef
import pkg_resources

# REMOVE
import wec.wec_visual
import time

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

        # Initialize lists for PTO types and constraints
        self.linear_ptos = []
        self.rotary_ptos = []
        self.pivot_joints = []
        self.groove_joints = []
        self.linear_ptos_data = []
        self.rotary_ptos_data = []

        # Initialize lists for mooring system
        self.fixed_bodies = []
        self.cable_bodies = []
        self.mooring_attachment_points = []

        # current iteration
        self.iter = 0
        self.display = wec.wec_visual.wec_visual()

    def add_body(self, body_shape, density, position, **kwargs):
        # Calculate properties of body
        radius = 0
        length = 0
        angle_offset = 0

        if body_shape is 'sphere':
            radius = kwargs['radius']
            volume = (4.0 / 3.0) * np.pi * np.power(radius, 3)
            mass = density * volume
            moment = 0.4 * mass * np.power(radius, 2)
            coefficients = []
        elif body_shape is 'cylinder':
            radius = kwargs['radius']
            length = kwargs['length']
            angle_offset = kwargs['angle_offset']
            volume = np.pi * np.power(radius, 2) * length
            mass = density * volume
            # Moment about central diameter
            moment = 0.25 * mass * np.power(radius, 2) + (1 / 12.) * mass * np.power(length, 2)
            coefficients = []

        # Create body for simulation
        temp_body = pm.Body(mass=mass, moment=moment)
        temp_body.position = position
        temp_body.angle = (angle_offset*np.pi)/180

        # Create shape for visual
        if body_shape is 'sphere':
            shape = pm.Circle(temp_body, kwargs['radius'])
        elif body_shape is 'cylinder':
            shape = pm.Poly(temp_body, [(length/2, radius/2), (length/2, -radius/2), (-length/2, radius/2), (-length/2, -radius/2)], radius=3)

        # Add body and shape to simulation and stored lists
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
                            "angle_offset": angle_offset,
                            "xyz": np.zeros((self.simulation_steps, 2)),
                            "last_velocity": np.zeros(2),
                            "last_position": np.zeros(2),
                            "shape": shape,
                            "body_shape": body_shape})

    def add_constrained_linear_pto(self, idxa, idxb, resting_length, stiffness, damping):
        # Create damped spring for simulation
        a = self.bodies[idxa]["body"]
        b = self.bodies[idxb]["body"]
        temp_pto = pm.constraint.DampedSpring(a, b, a.center_of_gravity, b.center_of_gravity,
                                              resting_length, stiffness, damping)
        temp_pto.collide_bodies = False
        temp_pto.error_bias = self.error_bias
        # Add damped spring to simulation and stored lists
        self.world.add(temp_pto)
        self.linear_ptos.append(temp_pto)
        self.linear_ptos_data.append({"Linear PTO": temp_pto,
                                      "idxa": idxa,
                                      "idxb": idxb,
                                      "resting_length": resting_length,
                                      "stiffness": stiffness,
                                      "damping": damping})

        # Create first groove to constrain linear motion
        temp_groove = pm.constraint.GrooveJoint(a, b, a.center_of_gravity,
                                                a.center_of_gravity + 2 * (b.position - a.position),
                                                b.center_of_gravity)
        temp_groove.collide_bodies = False
        temp_groove.error_bias = self.error_bias
        # Add first groove to simulation and stored list
        self.world.add(temp_groove)
        self.groove_joints.append(temp_groove)
        # Create second groove to constrain linear motion
        temp_groove = pm.constraint.GrooveJoint(b, a, b.center_of_gravity,
                                                b.center_of_gravity + 2 * (a.position - b.position),
                                                a.center_of_gravity)
        temp_groove.collide_bodies = False
        temp_groove.error_bias = self.error_bias
        # Add second groove to simulation and stored list
        self.world.add(temp_groove)
        self.groove_joints.append(temp_groove)

    # LUCAS: Lower tier operations go here
    def add_rotational_pto(self, idxa, idxb, rest_angle, stiffness, damping):
        # Create damped rotary spring for simulation
        a = self.bodies[idxa]["body"]
        b = self.bodies[idxb]["body"]
        temp_pto = pm.constraint.DampedRotarySpring(a, b, rest_angle, stiffness, damping)
        temp_pto.collide_bodies = False
        temp_pto.error_bias = self.error_bias
        # Add damped rotary spring to simulation and stored lists
        self.world.add(temp_pto)
        self.rotary_ptos.append(temp_pto)
        self.rotary_ptos_data.append({"Rotational PTO": temp_pto,
                                      "idxa": idxa,
                                      "idxb": idxb,
                                      "rest_angle": rest_angle,
                                      "stiffness": stiffness,
                                      "damping": damping})

        # Create a pivot joint
        acg = a.local_to_world(a.center_of_gravity)
        bcg = b.local_to_world(b.center_of_gravity)
        temp_pivot = pm.constraint.PivotJoint(a, b, 0.5 * (acg + bcg))
        temp_pivot.collide_bodies = False
        temp_pivot.error_bias = self.error_bias
        # Add pivot to simulation and stored list
        self.world.add(temp_pivot)
        self.pivot_joints.append(temp_pivot)

    def add_rotational_body(self, shape, density, position, idxa, idxb, rest_angle, stiffness, damping, **kwargs):
        self.add_body(shape, density, position, **kwargs)
        self.add_rotational_pto(idxa, idxb, rest_angle, stiffness, damping)

    def add_linear_body(self, shape, density, position, idxa, idxb, resting_length, stiffness, damping, **kwargs):
        self.add_body(shape, density, position, **kwargs)
        self.add_constrained_linear_pto(idxa, idxb, resting_length, stiffness, damping)

    def remove_body(self, index):
        # Create temporary storage of body of shape
        body = self.bodies[index]['body']
        shape = self.bodies[index]['shape']
        # Remove body and shape from simulation
        self.world.remove(body)
        self.world.remove(shape)
        # Clear information from body list
        del self.bodies[index]

    def remove_joint(self, index, joint_type):
        # Removing rotary pto and pivot joint
        if joint_type is 'rotational':
            # Create reference objects for spring and pivot
            temp_pto = self.rotary_ptos[index]
            temp_pivot = self.pivot_joints[index]

            # Delete from world
            self.world.remove(temp_pto)
            self.world.remove(temp_pivot)

            # Delete from storage lists
            del self.rotary_ptos[index]
            del self.rotary_ptos_data[index]
            del self.pivot_joints[index]
        # Removing linear pto and groove joint
        elif joint_type is 'linear':
            # Create reference objects for spring and grooves
            temp_pto = self.linear_ptos[index]
            temp_groove_a = self.groove_joints[index * 2]
            temp_groove_b = self.groove_joints[index * 2 + 1]

            # Delete from world
            self.world.remove(temp_pto)
            self.world.remove(temp_groove_a)
            self.world.remove(temp_groove_b)

            # Delete from storage lists
            del self.linear_ptos[index]
            del self.linear_ptos_data[index]
            del self.groove_joints[index * 2]
            del self.groove_joints[index * 2]

    def joint_reinstancing(self, body_index):
        joint_index = 0
        for pto in self.rotary_ptos_data:
            if pto["idxa"] is body_index or pto["idxb"] is body_index:
                self.remove_joint(joint_index, 'rotational')
                self.add_rotational_pto(pto["idxa"], pto["idxb"], pto["rest_angle"], pto["stiffness"], pto["damping"])
                temp_joint = self.rotary_ptos[-1]
                temp_joint_data = self.rotary_ptos_data[-1]
                temp_pivot = self.pivot_joints[-1]
                self.rotary_ptos.insert(joint_index, temp_joint)
                self.rotary_ptos_data.insert(joint_index, temp_joint_data)
                self.pivot_joints.insert(joint_index, temp_pivot)
                del self.rotary_ptos[-1]
                del self.rotary_ptos_data[-1]
                del self.pivot_joints[-1]
            joint_index += 1

        joint_index = 0
        for pto in self.linear_ptos_data:
            if pto["idxa"] is body_index or pto["idxb"] is body_index:
                self.remove_joint(joint_index, 'linear')
                pos_a = self.bodies[pto["idxa"]]["body"].position
                pos_b = self.bodies[pto["idxb"]]["body"].position
                resting_length = np.sqrt((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2)
                self.add_constrained_linear_pto(pto["idxa"], pto["idxb"], resting_length, pto["stiffness"], pto["damping"])
                temp_joint = self.linear_ptos[-1]
                temp_joint_data = self.linear_ptos_data[-1]
                temp_groove = self.groove_joints[-1]
                self.linear_ptos.insert(joint_index, temp_joint)
                self.linear_ptos_data.insert(joint_index, temp_joint_data)
                self.groove_joints.insert(joint_index, temp_groove)
                del self.linear_ptos[-1]
                del self.linear_ptos_data[-1]
                del self.groove_joints[-1]
            joint_index += 1

    def mooring_reinstancing(self, body_index):
        mooring_index = 0
        for cable in self.cable_bodies:
            if cable.a is body_index or cable.b is body_index:
                temp_fixed_body = self.fixed_bodies[mooring_index]
                pos_fixed = temp_fixed_body["body"].position
                pos_body = self.bodies[body_index]["body"].position
                resting_length = np.sqrt((pos_fixed[0] - pos_body[0]) ** 2 + (pos_fixed[1] - pos_body[1]) ** 2)

                self.remove_mooring_system(mooring_index)
                self.add_mooring_system(pos_fixed, body_index, resting_length, cable.stiffness, cable.damping)

                temp_fixed_body = self.fixed_bodies[-1]
                self.fixed_bodies.insert(mooring_index, temp_fixed_body)
                del self.fixed_bodies[-1]
                temp_cable = self.cable_bodies[-1]
                self.cable_bodies.insert(mooring_index, temp_cable)
                del self.cable_bodies[-1]
                temp_attachment_index = self.mooring_attachment_points[-1]
                self.mooring_attachment_points.insert(mooring_index, temp_attachment_index)
                del self.mooring_attachment_points[-1]

            mooring_index += 1

    def remove_body_with_joint(self, body_index, joint_index, joint_type):
        self.remove_joint(joint_index, joint_type)
        self.remove_body(body_index)

    def change_joint_type(self, index, joint_initial_type):
        # Change from initial joint type to opposite
        if joint_initial_type is 'rotational':
            # Use distance between bodies as resting length
            pos_a = self.bodies[self.rotary_ptos_data[index]["idxa"]]["body"].position
            pos_b = self.bodies[self.rotary_ptos_data[index]["idxb"]]["body"].position
            resting_length = np.sqrt((pos_a[0]-pos_b[0])**2 + (pos_a[1]-pos_b[1])**2)

            self.add_constrained_linear_pto(self.rotary_ptos_data[index]["idxa"],
                                            self.rotary_ptos_data[index]["idxb"],
                                            resting_length,
                                            self.rotary_ptos_data[index]["stiffness"],
                                            self.rotary_ptos_data[index]["damping"])
        elif joint_initial_type is 'linear':
            # Not sure if this angle should be 0 or something else
            rest_angle = 0
            self.add_rotational_pto(self.linear_ptos_data[index]["idxa"],
                                    self.linear_ptos_data[index]["idxb"],
                                    rest_angle,
                                    self.linear_ptos_data[index]["stiffness"],
                                    self.linear_ptos_data[index]["damping"])

        # Remove original joint
        self.remove_joint(index, joint_initial_type)

    def change_body_dimensions(self, index, **kwargs):
        temp_body = self.bodies[index]
        self.remove_body(index)
        self.add_body(temp_body['body_shape'], temp_body['density'], temp_body['body'].position, **kwargs)
        temp_body = self.bodies[-1]
        self.bodies.insert(index, temp_body)
        del self.bodies[-1]
        self.joint_reinstancing(index)
        self.mooring_reinstancing(index)

    def change_body_density(self, index, density):
        temp_body = self.bodies[index]
        self.remove_body(index)
        self.add_body(temp_body['body_shape'], density, temp_body['body'].position,
                      radius=temp_body['radius'], length=temp_body['length'])
        temp_body = self.bodies[-1]
        self.bodies.insert(index, temp_body)
        del self.bodies[-1]
        self.joint_reinstancing(index)
        self.mooring_reinstancing(index)

    def relocate_body_with_joint(self, body_index, joint_index, joint_type, new_position, attach_body_index):
        temp_body = self.bodies[body_index]
        shape = temp_body["body_shape"]
        density = temp_body["density"]
        radius = temp_body["radius"]
        length = temp_body["length"]
        self.add_body(shape, density, new_position, radius=radius, length=length)
        self.bodies.insert(body_index, self.bodies[-1])

        if joint_type is 'rotational':
            temp_joint = self.rotary_ptos_data[joint_index]
            self.remove_body_with_joint(body_index + 1, joint_index, joint_type)
            self.add_rotational_pto(body_index, attach_body_index, temp_joint["rest_angle"],
                                    temp_joint["stiffness"], temp_joint["damping"])

        elif joint_type is 'linear':
            temp_joint = self.linear_ptos_data[joint_index]
            self.remove_body_with_joint(body_index + 1, joint_index, joint_type)
            pos_a = self.bodies[body_index]["body"].position
            pos_b = self.bodies[attach_body_index]["body"].position
            resting_length = np.sqrt((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2)
            self.add_constrained_linear_pto(body_index, attach_body_index, resting_length,
                                            temp_joint["stiffness"], temp_joint["damping"])
        self.mooring_reinstancing(body_index)

    def swap_bodies(self, idxa, idxb):
        temp_body_a = self.bodies[idxa]
        temp_body_b = self.bodies[idxb]

        self.remove_body(idxa)
        self.add_body(temp_body_b['body_shape'], temp_body_b['density'], temp_body_a['body'].position,
                      radius=temp_body_b['radius'], length=temp_body_b['length'])
        temp_body = self.bodies[-1]
        self.bodies.insert(idxa, temp_body)
        del self.bodies[-1]

        self.remove_body(idxb)
        self.add_body(temp_body_a['body_shape'], temp_body_a['density'], temp_body_b['body'].position,
                      radius=temp_body_a['radius'], length=temp_body_a['length'])
        temp_body = self.bodies[-1]
        self.bodies.insert(idxb, temp_body)
        del self.bodies[-1]

        self.joint_reinstancing(idxa)
        self.joint_reinstancing(idxb)
        self.mooring_reinstancing(idxa)
        self.mooring_reinstancing(idxb)

    def add_mooring_system(self, position, body_index, resting_length, stiffness, damping):
        radius = 20

        # Create fixed body for simulation
        temp_body = pm.Body(body_type=pm.Body.STATIC)
        temp_body.position = position
        # Create shape for visual
        shape = pm.Circle(temp_body, radius)
        # Create cable body as spring constraint for simulation
        attach_body = self.bodies[body_index]["body"]
        temp_cable = pm.constraint.DampedSpring(attach_body, temp_body, attach_body.center_of_gravity,
                                                temp_body.center_of_gravity, resting_length, stiffness, damping)
        # Add fixed and cable bodies to simulation and stored lists
        self.world.add(temp_body)
        self.world.add(shape)
        self.world.add(temp_cable)
        self.fixed_bodies.append({"body": temp_body,
                                  "shape": shape,
                                  "position": position})
        self.cable_bodies.append(temp_cable)
        self.mooring_attachment_points.append(body_index)

    def remove_mooring_system(self, index):
        # Create temporary storage of fixed body, shape, and cable
        body = self.fixed_bodies[index]['body']
        shape = self.fixed_bodies[index]['shape']
        cable_spring = self.cable_bodies[index]

        # Remove body and shape from simulation
        self.world.remove(body)
        self.world.remove(shape)
        self.world.remove(cable_spring)

        # Clear information from lists
        del self.fixed_bodies[index]
        del self.cable_bodies[index]
        del self.mooring_attachment_points[index]

    def relocate_mooring_cable_attachment(self, mooring_index, body_index):
        temp_fixed_body = self.fixed_bodies[mooring_index]
        temp_cable = self.cable_bodies[mooring_index]

        pos_fixed = temp_fixed_body["body"].position
        pos_body = self.bodies[body_index]["body"].position
        resting_length = np.sqrt((pos_fixed[0] - pos_body[0]) ** 2 + (pos_fixed[1] - pos_body[1]) ** 2)

        self.remove_mooring_system(mooring_index)
        self.add_mooring_system(pos_fixed, body_index, resting_length, temp_cable.stiffness, temp_cable.damping)

        temp_fixed_body = self.fixed_bodies[-1]
        self.fixed_bodies.insert(mooring_index, temp_fixed_body)
        del self.fixed_bodies[-1]
        temp_cable = self.cable_bodies[-1]
        self.cable_bodies.insert(mooring_index, temp_cable)
        del self.cable_bodies[-1]
        temp_attachment_index = self.mooring_attachment_points[-1]
        self.mooring_attachment_points.insert(mooring_index, temp_attachment_index)
        del self.mooring_attachment_points[-1]

    def relocate_mooring_fixed_body(self, mooring_index, position):
        temp_cable = self.cable_bodies[mooring_index]
        temp_body_index = self.mooring_attachment_points[mooring_index]
        temp_body = self.bodies[temp_body_index]

        pos_fixed = position
        pos_body = temp_body["body"].position
        resting_length = np.sqrt((pos_fixed[0] - pos_body[0]) ** 2 + (pos_fixed[1] - pos_body[1]) ** 2)

        self.remove_mooring_system(mooring_index)
        self.add_mooring_system(pos_fixed, temp_body_index, resting_length, temp_cable.stiffness, temp_cable.damping)

        temp_fixed_body = self.fixed_bodies[-1]
        self.fixed_bodies.insert(mooring_index, temp_fixed_body)
        del self.fixed_bodies[-1]
        temp_cable = self.cable_bodies[-1]
        self.cable_bodies.insert(mooring_index, temp_cable)
        del self.cable_bodies[-1]
        temp_attachment_index = self.mooring_attachment_points[-1]
        self.mooring_attachment_points.insert(mooring_index, temp_attachment_index)
        del self.mooring_attachment_points[-1]

    # LUCAS: Put lower tier operations above here

    # LUCAS: Higher tier operations will go here eventually too. These should follow the function definitions as defined
    #        in the abstract base solution class.

    ### Everything below here is for evaluation

    def add_buoyant_force(self):
        for body in self.bodies:
            r = body["radius"]
            if r + body["body"].position[1] < 0:
                displaced_mass = body["volume"] * self.rho_w
            elif body["body"].position[1] - r > 0:
                displaced_mass = 0
            else:
                h = r - body["body"].position[1]
                c = np.sqrt(h * (2 * r - h))
                displaced_mass = self.rho_w * (np.pi / 6) * h * (3 * c * c + h * h)

            faddl = -displaced_mass * self.world.gravity
            body["body"].apply_force_at_local_point(faddl, body["body"].center_of_gravity)

    def add_excitation_force(self):
        for body in self.bodies:
            if body['body'].position[1] > -10:
                body["body"].apply_force_at_local_point(
                    (0, body['mass'] * np.sin(self.iter / 100 + body['body'].position[0])),
                    body["body"].center_of_gravity)

    def add_radiative_force(self):
        for body in self.bodies:
            body["body"].apply_force_at_local_point((0, 0), body["body"].center_of_gravity)

    def add_viscous_force(self):
        for body in self.bodies:
            # Compute viscous drag of sphere perpendicular to force,
            Cd = 0.47
            v = body["body"].velocity
            A = np.pi * np.power(body["radius"], 2)
            f = -0.5 * self.rho_w * Cd * A * v * np.linalg.norm(v)

            body["body"].apply_force_at_local_point(f, body["body"].center_of_gravity)

    def pull_position_data(self):
        for i in range(len(self.bodies)):
            self.bodies[i]['xyz'][self.iter, 0] = self.bodies[i]['body'].position[0]
            self.bodies[i]['xyz'][self.iter, 1] = self.bodies[i]['body'].position[1]

    def evaluate(self):
        energy = np.zeros(2)

        while self.iter < self.simulation_steps:

            self.display.display(self)

            # Add forces
            self.add_buoyant_force()
            self.add_excitation_force()
            # self.add_radiative_force()
            self.add_viscous_force()

            # Step the simulation forward
            self.world.step(self.simulation_dt)

            # Perform corrections
            for body in self.bodies:
                body['body'].velocity = 0.5 * (body['body'].velocity + body['last_velocity'])
                dxy = body['body'].position - body['last_position']
                dv = np.linalg.norm(body['body'].velocity) / np.linalg.norm(body['last_velocity'])
                if np.isinf(dv) or np.isnan(dv):
                    dv = 1.0
                body['body'].position = body['last_position'] + dv * dxy

                body['last_velocity'] = body['body'].velocity
                body['last_position'] = body['body'].position
                # print(dv, dxy, body['last_position'], body['last_velocity'])

            # Pull position data
            self.pull_position_data()

            # Track PTO energy extraction
            for pto in self.linear_ptos:
                relative_velocity = np.linalg.norm(pto.a.velocity - pto.b.velocity)
                energy[0] += np.power(relative_velocity, 2) * pto.damping * self.simulation_dt
            for pto in self.rotary_ptos:
                relative_velocity = np.abs(pto.a.angular_velocity - pto.b.angular_velocity)
                energy[1] += np.power(relative_velocity, 2) * pto.damping * self.simulation_dt

            self.iter += 1

            time.sleep(.01)

        self.iter = 0

        # Calculate power
        self.power = energy / (self.simulation_dt * self.simulation_steps)

        # Calculate mass
        self.mass = 0
        for body in self.bodies:
            self.mass += body['mass']

        # Calculate number of pieces
        self.pieces = len(self.linear_ptos) + len(self.rotary_ptos) + len(self.bodies)

        return [self.mass, -self.power, self.pieces]

    def __deepcopy__(self):
        asdf = 1
