import numpy as np
import pymunk as pm

from heuristic_bursts.abstract_base_solution import AbstractBaseSolution

import wec.spectrum as sp
import wec.excitation_forces as ef
import pkg_resources
import random

import copy

import wec.wec_visual
import time


class WEC(AbstractBaseSolution):
    simulation_dt = 0.01
    simulation_steps = 2000
    initial_steps = 7000
    error_bias = pow(1.0 - 0.1, 10.0)
    spectrum = sp.Spectrum('bretschneider', fp=0.5, Hm0=2)
    forces = ef.ExcitationForces()
    forces.load_model(pkg_resources.resource_filename('wec', 'data/full_network_structure.yml'),
                      pkg_resources.resource_filename('wec', 'data/full_network_weights.h5'))
    rho_w = 1000
    gravity = -9.81
    number_of_metrics = 3
    sea_level = 0
    pto_size = 1

    def __init__(self):
        #
        self.world = pm.Space()
        self.world.gravity = (0, -9.81)
        self.world.damping = 0.95
        self.stable_system = True

        # Initialize center of gravity of system
        self.center_of_gravity = (0, 0)

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
        self.addition_locations = []
        self.deletable_bodies = []

        # Initialize lists for mooring system
        self.fixed_bodies = []
        self.cable_bodies = []
        self.mooring_attachment_points = []

        # Current iteration
        self.iter = 0
        self.display_visual = False

        # List of applied rules
        self.applied_rules = []

        # Initialize design
        self.add_body('sphere', 500.0, (0, 0), radius=2)
        self.lowtier_rule_perform(1, location=(0, 0), radius=2, stiffness=1000, damping=1000000)
        self.lowtier_rule_perform(1, location=(0, 2), radius=2, stiffness=1000, damping=1000000)
        self.lowtier_rule_perform(2, location=(0, 3), radius=2, stiffness=1000, damping=1000000)

        #PELAMIS
        # self.lowtier_rule_perform(2, location=(0, 0), radius=1, stiffness=1000, damping=1000000)
        # self.lowtier_rule_perform(2, location=(1, 3), radius=1.6, stiffness=1000, damping=1000000)
        # self.lowtier_rule_perform(2, location=(2, 2), radius=1.2, stiffness=1000, damping=1000000)

        #POWERBUOY
        # self.lowtier_rule_perform(2, location=(3, 3), radius=2, stiffness=1000, damping=1000000)

        self.applied_rules = []

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
                            "xyz": np.zeros((self.simulation_steps + self.initial_steps, 2)),
                            "last_velocity": np.zeros(2),
                            "last_position": np.zeros(2),
                            "shape": shape,
                            "body_shape": body_shape,
                            "voxels": []})

    def add_constrained_linear_pto(self, idxa, idxb, resting_length, stiffness, damping):
        # Create damped spring for simulation
        a = self.bodies[idxa]["body"]
        b = self.bodies[idxb]["body"]
        temp_pto = pm.constraint.DampedSpring(a, b, a.center_of_gravity, b.center_of_gravity,
                                              resting_length, stiffness, damping)
        temp_pto.collide_bodies = True
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
        temp_groove.collide_bodies = True
        temp_groove.error_bias = self.error_bias
        # Add first groove to simulation and stored list
        self.world.add(temp_groove)
        self.groove_joints.append(temp_groove)
        # Create second groove to constrain linear motion
        temp_groove = pm.constraint.GrooveJoint(b, a, b.center_of_gravity,
                                                b.center_of_gravity + 2 * (a.position - b.position),
                                                a.center_of_gravity)
        temp_groove.collide_bodies = True
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
        temp_pto.collide_bodies = True
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
        temp_pivot.collide_bodies = True
        temp_pivot.error_bias = self.error_bias
        # Add pivot to simulation and stored list
        self.world.add(temp_pivot)
        self.pivot_joints.append(temp_pivot)

    def add_rotational_body(self, shape, density, position, idxa, rest_angle, stiffness, damping, **kwargs):
        self.add_body(shape, density, position, **kwargs)
        idxb = len(self.bodies) - 1
        self.add_rotational_pto(idxa, idxb, rest_angle, stiffness, damping)

    def add_linear_body(self, shape, density, position, idxa, stiffness, damping, **kwargs):
        self.add_body(shape, density, position, **kwargs)
        idxb = len(self.bodies) - 1
        pos_a = self.bodies[idxa]["body"].position
        pos_b = self.bodies[idxb]["body"].position
        resting_length = np.sqrt((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2)
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

    def repositioning(self, index, initial_body, new_body):
        self.bodies_shifted_upwards_out_of_water = False
        for relative_position in range(0, 4):
            self.reposition_branch(index, relative_position, initial_body, new_body)

        if self.bodies_shifted_upwards_out_of_water:
            for i in range(0, len(self.bodies)):
                temp_body_index = i
                temp_body = self.bodies[i]
                temp_body_pos = temp_body['body'].position
                temp_body_pos[0] = temp_body_pos[0]
                temp_body_pos[1] -= self.rad_diff
                self.remove_body(i)
                self.add_body(temp_body['body_shape'], temp_body['density'], temp_body_pos,
                              radius=temp_body['radius'], length=temp_body['length'],
                              angle_offset=temp_body['angle_offset'])
                self.bodies.insert(i, self.bodies[-1])
                del self.bodies[-1]
                self.joint_reinstancing(i)
                self.mooring_reinstancing(i)

    def reposition_branch(self, body_index, relative_position, initial_body, new_body):
        old_pos = initial_body['body'].position
        R1 = initial_body['radius']
        R2 = new_body['radius']
        self.rad_diff = R2 - R1

        bodies_to_check = []
        bodies_to_shift = []

        for i in range(0, len(self.bodies)):
            test_body = self.bodies[i]
            test_body_pos = test_body['body'].position
            test_body_radius = test_body['radius']
            distance = np.sqrt((old_pos[0] - test_body_pos[0]) ** 2 + (old_pos[1] - test_body_pos[1]) ** 2)
            if distance <= R1 + test_body_radius + self.pto_size + 0.05 and \
                            distance >= R1 + test_body_radius + self.pto_size - 0.05:
                if test_body_pos[1] == old_pos[1] and test_body_pos[0] > old_pos[0] and relative_position == 0:
                    bodies_to_check.append(i)
                    bodies_to_shift.append(i)
                    break
                elif test_body_pos[0] == old_pos[0] and test_body_pos[1] > old_pos[1] and relative_position == 1:
                    bodies_to_check.append(i)
                    bodies_to_shift.append(i)
                    break
                elif test_body_pos[1] == old_pos[1] and test_body_pos[0] < old_pos[0] and relative_position == 2:
                    bodies_to_check.append(i)
                    bodies_to_shift.append(i)
                    break
                elif test_body_pos[0] == old_pos[0] and test_body_pos[1] < old_pos[1] and relative_position == 3:
                    bodies_to_check.append(i)
                    bodies_to_shift.append(i)
                    break

        while len(bodies_to_check) > 0:
            a = bodies_to_check[0]
            for b in range(0, len(self.bodies)):
                for pto in self.rotary_ptos_data:
                    if ((pto['idxa'] == a and pto['idxb'] == b) or (pto['idxa'] == b and pto['idxb'] == a)) and (b not in bodies_to_check) and (b not in bodies_to_shift) and b != body_index:
                        bodies_to_check.append(b)
                        bodies_to_shift.append(b)
                for pto in self.linear_ptos_data:
                    if ((pto['idxa'] == a and pto['idxb'] == b) or (pto['idxa'] == b and pto['idxb'] == a)) and (b not in bodies_to_check) and (b not in bodies_to_shift) and b != body_index:
                        bodies_to_check.append(b)
                        bodies_to_shift.append(b)
            del bodies_to_check[0]

        while len(bodies_to_shift) > 0:
            temp_body_index = bodies_to_shift[0]
            temp_body = self.bodies[temp_body_index]
            temp_body_pos = temp_body['body'].position

            if relative_position == 0:
                temp_body_pos[0] = temp_body_pos[0] + self.rad_diff
            elif relative_position == 1:
                temp_body_pos[1] = temp_body_pos[1] + self.rad_diff
                self.bodies_shifted_upwards_out_of_water = True
            elif relative_position == 2:
                temp_body_pos[0] = temp_body_pos[0] - self.rad_diff
            elif relative_position == 3:
                temp_body_pos[1] = temp_body_pos[1] - self.rad_diff

            self.remove_body(temp_body_index)
            self.add_body(temp_body['body_shape'], temp_body['density'], temp_body_pos,
                          radius=temp_body['radius'], length=temp_body['length'], angle_offset=temp_body['angle_offset'])
            self.bodies.insert(temp_body_index, self.bodies[-1])
            del self.bodies[-1]
            self.joint_reinstancing(temp_body_index)
            self.mooring_reinstancing(temp_body_index)
            del bodies_to_shift[0]

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
                temp_groove_a = self.groove_joints[-2]
                temp_groove_b = self.groove_joints[-1]
                self.linear_ptos.insert(joint_index, temp_joint)
                self.linear_ptos_data.insert(joint_index, temp_joint_data)
                self.groove_joints.insert(joint_index*2, temp_groove_a)
                self.groove_joints.insert(joint_index*2 + 1, temp_groove_b)
                del self.linear_ptos[-1]
                del self.linear_ptos_data[-1]
                del self.groove_joints[-1]
                del self.groove_joints[-1]
            joint_index += 1

    def joint_index_shift(self, body_index):
        for pto in self.rotary_ptos_data:
            if pto["idxa"] > body_index:
                pto["idxa"] -= 1
            if pto["idxb"] > body_index:
                pto["idxb"] -= 1
        for pto in self.linear_ptos_data:
            if pto["idxa"] > body_index:
                pto["idxa"] -= 1
            if pto["idxb"] > body_index:
                pto["idxb"] -= 1

    def mooring_index_shift(self, body_index):
        for mooring in self.mooring_attachment_points:
            if mooring > body_index:
                mooring -= 1

    def mooring_reinstancing(self, body_index):
        mooring_index = 0
        for cable in self.cable_bodies:
            attachment_body = self.mooring_attachment_points[mooring_index]
            if attachment_body is body_index:
                temp_fixed_body = self.fixed_bodies[mooring_index]
                self.remove_mooring_system(mooring_index)
                self.add_mooring_system(temp_fixed_body["body"].position, body_index, cable.stiffness, cable.damping)

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
        self.joint_index_shift(body_index)
        if len(self.mooring_attachment_points) > 0:
            for mooring_index in range(0, len(self.mooring_attachment_points)):
                attachment_body = self.mooring_attachment_points[mooring_index]
                if attachment_body == body_index:
                    self.remove_mooring_system(mooring_index)
                    break
        self.mooring_index_shift(body_index)

    def change_joint_type(self, index, joint_initial_type):
        # Change from initial joint type to opposite
        if joint_initial_type is 'rotational':
            # Use distance between bodies as resting length
            pos_a = self.bodies[self.rotary_ptos_data[index]["idxa"]]["body"].position
            pos_b = self.bodies[self.rotary_ptos_data[index]["idxb"]]["body"].position
            resting_length = np.sqrt((pos_a[0]-pos_b[0])**2 + (pos_a[1]-pos_b[1])**2) + self.pto_size

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
        initial_body = temp_body
        self.remove_body(index)
        self.add_body(temp_body['body_shape'], temp_body['density'], temp_body['body'].position, **kwargs)
        temp_body = self.bodies[-1]
        new_body = temp_body
        self.bodies.insert(index, temp_body)
        del self.bodies[-1]
        self.repositioning(index, initial_body, new_body)

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
        radius = temp_body["radius"]
        length = temp_body["length"]
        angle_offset = temp_body["angle_offset"]

        if radius + new_position[1] < self.sea_level:
            density = 1000
        elif new_position[1] - radius > self.sea_level:
            density = 350
        else:
            h = radius - new_position[1] + self.sea_level
            c = np.sqrt(h * (2 * radius - h))
            density = (self.rho_w * (np.pi / 6) * h * (3 * c * c + h * h)) / \
                      ((4.0 / 3.0) * np.pi * np.power(radius, 3))

        if joint_type is 'rotational':
            temp_joint = self.rotary_ptos_data[joint_index]
            self.remove_joint(joint_index, joint_type)
            self.remove_body(body_index)
            self.add_body(shape, density, new_position, radius=radius, length=length, angle_offset=angle_offset)
            temp_body = self.bodies[-1]
            self.bodies.insert(body_index, temp_body)
            del self.bodies[-1]
            self.add_rotational_pto(attach_body_index, body_index, temp_joint["rest_angle"],
                                    temp_joint["stiffness"], temp_joint["damping"])
            temp_joint = self.rotary_ptos[-1]
            temp_joint_data = self.rotary_ptos_data[-1]
            temp_pivot = self.pivot_joints[-1]
            self.rotary_ptos.insert(joint_index, temp_joint)
            self.rotary_ptos_data.insert(joint_index, temp_joint_data)
            self.pivot_joints.insert(joint_index, temp_pivot)
            del self.rotary_ptos[-1]
            del self.rotary_ptos_data[-1]
            del self.pivot_joints[-1]

        elif joint_type is 'linear':
            temp_joint = self.linear_ptos_data[joint_index]
            self.remove_joint(joint_index, joint_type)
            self.remove_body(body_index)
            self.add_body(shape, density, new_position, radius=radius, length=length, angle_offset=angle_offset)
            temp_body = self.bodies[-1]
            self.bodies.insert(body_index, temp_body)
            del self.bodies[-1]
            pos_a = self.bodies[body_index]["body"].position
            pos_b = self.bodies[attach_body_index]["body"].position
            resting_length = np.sqrt((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2) + self.pto_size
            self.add_constrained_linear_pto(body_index, attach_body_index, resting_length,
                                            temp_joint["stiffness"], temp_joint["damping"])
            temp_joint = self.linear_ptos[-1]
            temp_joint_data = self.linear_ptos_data[-1]
            temp_groove_a = self.groove_joints[-2]
            temp_groove_b = self.groove_joints[-1]
            self.linear_ptos.insert(joint_index, temp_joint)
            self.linear_ptos_data.insert(joint_index, temp_joint_data)
            self.groove_joints.insert(joint_index * 2, temp_groove_a)
            self.groove_joints.insert(joint_index * 2 + 1, temp_groove_b)
            del self.linear_ptos[-1]
            del self.linear_ptos_data[-1]
            del self.groove_joints[-1]
            del self.groove_joints[-1]
        self.mooring_reinstancing(body_index)

    def swap_bodies(self, idxa, idxb):
        temp_body_a = self.bodies[idxa]
        temp_body_b = self.bodies[idxb]
        temp_body_a_pos = temp_body_a['body'].position
        temp_body_b_pos = temp_body_b['body'].position
        initial_body_a = temp_body_a
        initial_body_b = temp_body_b

        if temp_body_b['radius'] + temp_body_a_pos[1] < self.sea_level:
            density = 1000
        elif temp_body_a_pos[1] - temp_body_b['radius'] > self.sea_level:
            density = 350
        else:
            h = temp_body_b['radius'] - temp_body_a_pos[1] + self.sea_level
            c = np.sqrt(h * (2 * temp_body_b['radius'] - h))
            density = (self.rho_w * (np.pi / 6) * h * (3 * c * c + h * h)) / \
                      ((4.0 / 3.0) * np.pi * np.power(temp_body_b['radius'], 3))
        self.remove_body(idxa)
        self.add_body(temp_body_b['body_shape'], density, temp_body_a['body'].position,
                      radius=temp_body_b['radius'], length=temp_body_b['length'], angle_offset=temp_body_b['angle_offset'])
        temp_body = self.bodies[-1]
        new_body_a = temp_body
        self.bodies.insert(idxa, temp_body)
        del self.bodies[-1]

        if temp_body_a['radius'] + temp_body_b_pos[1] < self.sea_level:
            density = 1000
        elif temp_body_b_pos[1] - temp_body_a['radius'] > self.sea_level:
            density = 350
        else:
            h = temp_body_a['radius'] - temp_body_b_pos[1] + self.sea_level
            c = np.sqrt(h * (2 * temp_body_a['radius'] - h))
            density = (self.rho_w * (np.pi / 6) * h * (3 * c * c + h * h)) / \
                      ((4.0 / 3.0) * np.pi * np.power(temp_body_a['radius'], 3))
        self.remove_body(idxb)
        self.add_body(temp_body_a['body_shape'], density, temp_body_b['body'].position,
                      radius=temp_body_a['radius'], length=temp_body_a['length'], angle_offset=temp_body_a['angle_offset'])
        temp_body = self.bodies[-1]
        new_body_b = temp_body
        self.bodies.insert(idxb, temp_body)
        del self.bodies[-1]

        self.repositioning(idxa, initial_body_a, new_body_a)
        self.repositioning(idxb, initial_body_b, new_body_b)
        self.joint_reinstancing(idxa)
        self.joint_reinstancing(idxb)
        self.mooring_reinstancing(idxa)
        self.mooring_reinstancing(idxb)

    def change_joint_coefficients(self, index, joint_type, **kwargs):
        # Extract current information for PTO
        if joint_type is 'rotational':
            temp_joint = self.rotary_ptos_data[index]
            rest_angle = temp_joint['rest_angle']
        elif joint_type is 'linear':
            temp_joint = self.linear_ptos_data[index]
            resting_length = temp_joint['resting_length']

        idxa = temp_joint['idxa']
        idxb = temp_joint['idxb']
        if kwargs.get('stiffness') is not None:
            stiffness = kwargs['stiffness']
        else:
            stiffness = temp_joint['stiffness']
        if kwargs.get('damping') is not None:
            damping = kwargs['damping']
        else:
            damping = temp_joint['damping']

        # print('joint index: ', index)
        # print('joint type: ', joint_type)
        # print('idxa: ', idxa)
        # print('idxb: ', idxb)
        # print('stiffness: ', temp_joint['stiffness'])
        # print('damping: ', temp_joint['damping'])
        # print(' ')

        # Remove original joint
        self.remove_joint(index, joint_type)

        # Add new joint and reposition within lists to return to original index
        if joint_type is 'rotational':
            self.add_rotational_pto(idxa, idxb, rest_angle, stiffness, damping)
            temp_joint = self.rotary_ptos[-1]
            temp_joint_data = self.rotary_ptos_data[-1]
            temp_pivot = self.pivot_joints[-1]
            self.rotary_ptos.insert(index, temp_joint)
            self.rotary_ptos_data.insert(index, temp_joint_data)
            self.pivot_joints.insert(index, temp_pivot)
            del self.rotary_ptos[-1]
            del self.rotary_ptos_data[-1]
            del self.pivot_joints[-1]
            # print(self.rotary_ptos_data[index])
        elif joint_type is 'linear':
            self.add_constrained_linear_pto(idxa, idxb, resting_length, stiffness, damping)
            temp_joint = self.linear_ptos[-1]
            temp_joint_data = self.linear_ptos_data[-1]
            temp_groove_a = self.groove_joints[-2]
            temp_groove_b = self.groove_joints[-1]
            self.linear_ptos.insert(index, temp_joint)
            self.linear_ptos_data.insert(index, temp_joint_data)
            self.groove_joints.insert(index * 2, temp_groove_a)
            self.groove_joints.insert(index * 2 + 1, temp_groove_b)
            del self.linear_ptos[-1]
            del self.linear_ptos_data[-1]
            del self.groove_joints[-1]
            del self.groove_joints[-1]
            # print(self.linear_ptos_data[index])

    def add_mooring_system(self, position, body_index, stiffness, damping):
        radius = 20

        pos_a = position
        pos_b = self.bodies[body_index]["body"].position
        resting_length = np.sqrt((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2)

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

        self.remove_mooring_system(mooring_index)
        self.add_mooring_system(temp_fixed_body["body"].position, body_index, temp_cable.stiffness, temp_cable.damping)

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
        # temp_body = self.bodies[temp_body_index]

        self.remove_mooring_system(mooring_index)
        self.add_mooring_system(position, temp_body_index, temp_cable.stiffness, temp_cable.damping)

        temp_fixed_body = self.fixed_bodies[-1]
        self.fixed_bodies.insert(mooring_index, temp_fixed_body)
        del self.fixed_bodies[-1]
        temp_cable = self.cable_bodies[-1]
        self.cable_bodies.insert(mooring_index, temp_cable)
        del self.cable_bodies[-1]
        temp_attachment_index = self.mooring_attachment_points[-1]
        self.mooring_attachment_points.insert(mooring_index, temp_attachment_index)
        del self.mooring_attachment_points[-1]

    def locate_center_of_gravity(self):
        total_x_weighted = 0
        total_y_weighted = 0
        total_mass = 0

        for index in range(0, len(self.bodies)):
            temp_body = self.bodies[index]
            temp_pos = temp_body['body'].position
            temp_mass = temp_body['mass']
            total_x_weighted += temp_pos[0]*temp_mass
            total_y_weighted += temp_pos[1]*temp_mass
            total_mass += temp_mass

        self.center_of_gravity = (total_x_weighted / total_mass, total_y_weighted / total_mass)

    # Start of higher-tier functions operations

    def create_initial_design(self, **kwargs):
        # Select number of low-tier rules to apply to generate initial design
        min_num_rules = 1
        max_num_rules = 30
        if kwargs.get('num_rules') is not None:
            num_rules = kwargs['num_rules']
        else:
            num_rules = random.randint(min_num_rules, max_num_rules)

        # Randomly select low-tier rules to apply to generate design
        for i in range(1, num_rules):
            rule = self.lowtier_rule_select()
            self.lowtier_rule_perform(rule)
            print('Validity: ', self.is_valid())
        self.applied_rules.append('H1')

    def change_design_scale(self, **kwargs):
       # Determine scale multiplier to be applied to size of design
        min_multiplier = 0.5
        max_multiplier = 2.0

        # Check to see if input multiplier value will result in a size that is not feasible due to small size
        valid_multiplier = False
        if kwargs.get('multiplier') is not None:
            multiplier = kwargs['multiplier']
            for index in range(0, len(self.bodies)):
                potential_radius = round((self.bodies[index]['radius'] * multiplier), 1)
                if potential_radius < 1.0 or potential_radius > 5.0:
                    valid_multiplier = False
                    break
                else:
                    valid_multiplier = True

        while not valid_multiplier:
            multiplier = round(random.uniform(min_multiplier, max_multiplier), 1)
            for index in range(0, len(self.bodies)):
                potential_radius = round((self.bodies[index]['radius'] * multiplier), 1)
                if potential_radius < 1.0 or potential_radius > 5.0:
                    valid_multiplier = False
                    break
                else:
                    valid_multiplier = True

        self.applied_rules.append('H4')

        # print(multiplier)

        for index in range(0, len(self.bodies)):
            body = self.bodies[index]
            radius = round((body['radius'] * multiplier), 1)
            length = int(body['length'] * multiplier)
            angle_offset = int(body['angle_offset'] * multiplier)
            self.change_body_dimensions(index, radius=radius, length=length, angle_offset=angle_offset)

    def increase_or_decrease_complexity(self, **kwargs):
        # Determine if device will become more complex or less complex
        increase = False
        decrease = False
        # Check kwargs to see if specific input was given
        if kwargs.get('complexity_change_type') is not None:
            decision = kwargs['complexity_change_type']
            if decision is 'increase':
                increase = True
            elif decision is 'decrease':
                decrease = True
        else:
            type_selection = random.randint(0, 1)
            if type_selection == 0:
                increase = True
            elif type_selection == 1:
                decrease = True

        # Determine length of rule
        min_length = 1
        max_length = 3
        # Check kwargs to see if specific input was given
        if kwargs.get('rule_length') is not None:
            rule_length = kwargs['rule_length']
        else:
            rule_length = random.randint(min_length, max_length)

        # Select direction to expand
        # Check kwargs to see if specific input was given
        if kwargs.get('branch_direction') is not None:
            branch_direction = kwargs['branch_direction']
        else:
            branch_direction_select = random.randint(0, 2)
            if branch_direction_select == 0:
                branch_direction = 'both'
            elif branch_direction_select == 1:
                branch_direction = 'x'
            elif branch_direction_select == 2:
                branch_direction = 'y'

        # Specify which positions relative to a body are allowed for expansion, given branch direction
        if branch_direction is 'both':
            valid_locations = [0, 1, 2, 3]
        elif branch_direction is 'x':
            valid_locations = [0, 2]
        elif branch_direction is 'y':
            valid_locations = [1, 3]

        self.rule_check()

        if increase:
            rule_possible = False
            for check in self.addition_locations:
                if check[1] in valid_locations:
                    rule_possible = True
                    break
            if rule_possible:
                left_to_check = copy.deepcopy(self.addition_locations)
                next_location = (-1, -1)
                while next_location[1] not in valid_locations and len(left_to_check) > 0:
                    check_index = random.randint(0, len(left_to_check)-1)
                    next_location = left_to_check[check_index]
                    del left_to_check[check_index]

                for n in range(0, rule_length):
                    rule_possible = False
                    for check in self.addition_locations:
                        if check[1] in valid_locations and check[0] == next_location[0]:
                            rule_possible = True
                            break
                    if rule_possible:
                        body_type = random.randint(0, 1)
                        if body_type == 0:
                            self.lowtier_rule_perform(1, location=next_location)
                            # Remove the last addition to list of applied rules to stop list from showing low-tier rule applied
                            del self.applied_rules[-1]
                        elif body_type == 1:
                            self.lowtier_rule_perform(2, location=next_location)
                            # Remove the last addition to list of applied rules to stop list from showing low-tier rule applied
                            del self.applied_rules[-1]
                        self.rule_check()
                        left_to_check = copy.deepcopy(self.addition_locations)
                        added_body_index = len(self.bodies) - 1
                        while (next_location[0] != added_body_index or (next_location[0] == added_body_index and next_location[1] not in valid_locations)) and len(left_to_check) > 0:
                            check_index = random.randint(0, len(left_to_check) - 1)
                            next_location = left_to_check[check_index]
                            del left_to_check[check_index]
                    else:
                        break
                self.applied_rules.append('H2')

        elif decrease and len(self.deletable_bodies) > 0:
            # Pick starting point for chain to delete from available deletable bodies
            next_delete = self.deletable_bodies[random.randint(0, len(self.deletable_bodies) - 1)]
            for n in range(0, rule_length):
                # print(self.deletable_bodies)
                if len(self.bodies) > 1:
                    temp_pto = (next_delete[1], next_delete[2])
                    if temp_pto[1] is 'rotational':
                        temp_pto = self.rotary_ptos_data[temp_pto[0]]
                        if temp_pto['idxa'] == next_delete[0]:
                            temp_next_delete = temp_pto['idxb']
                        elif temp_pto['idxb'] == next_delete[0]:
                            temp_next_delete = temp_pto['idxa']
                    elif temp_pto[1] is 'linear':
                        temp_pto = self.linear_ptos_data[temp_pto[0]]
                        if temp_pto['idxa'] == next_delete[0]:
                            temp_next_delete = temp_pto['idxb']
                        elif temp_pto['idxb'] == next_delete[0]:
                            temp_next_delete = temp_pto['idxa']

                    self.lowtier_rule_perform(3, removal=next_delete)
                    # Remove the last addition to list of applied rules to stop list from showing low-tier rule applied
                    del self.applied_rules[-1]

                    # Lower index of temp next delete if the deleted body had a lower index
                    if temp_next_delete > next_delete[0]:
                        temp_next_delete -= 1

                    if len(self.bodies) > 1:
                        self.rule_check()
                        for body in self.deletable_bodies:
                            if body[0] == temp_next_delete:
                                continue_rule = True
                                break
                            else:
                                continue_rule = False
                    else:
                        continue_rule = False

                    next_delete = (-1, -1, 'rotational')
                    if continue_rule:
                        while next_delete[0] is not temp_next_delete:
                            next_delete = self.deletable_bodies[random.randint(0, len(self.deletable_bodies) - 1)]
                    else:
                        break
            self.applied_rules.append('H3')

    def increase_symmetry(self):
        # TODO: Try to branch out and increase symmetry from center of gravity
        self.locate_center_of_gravity()

    def replicate_pattern(self, **kwargs):
        # 1. Choose length of pattern
        # 2. Choose number of patterns
        # 3. Choose initial body
        # 4. Copy body data
        # 5. If attached bodies are already in pattern, move to different body.
        #    If none left, jump ahead. Else copy next body data and location to last
        # 6. Choose to attach to left or right side of system
        # 7. Identify body furthest to attachment side
        # 8. Calculate shift in original position of patterned body to correctly position
        # 9. Set new positions for all bodies in pattern to shifted values and add bodies to world
        # 10. Add ptos to connect bodies in same orientation they previously were
        # 11. Replicate pattern for number of patterns on same side of system

        # Defined set maximum values for pattern
        max_bodies_in_pattern = 4
        max_num_patterns = 2

        # Select bodies in pattern and number of patterns
        # Check kwargs to see if specific input was given
        if kwargs.get('bodies_in_pattern') is not None:
            bodies_in_pattern = kwargs['bodies_in_pattern']
        else:
            bodies_in_pattern = random.randint(1, min(max_bodies_in_pattern, len(self.bodies)))
        if kwargs.get('num_patterns') is not None:
            num_patterns = kwargs['num_patterns']
        else:
            num_patterns = random.randint(1, max_num_patterns)

        # print("bodies in pattern: ", bodies_in_pattern)

        # Select which body to begin pattern
        # Check kwargs to see if specific input was given
        if kwargs.get('starting_body_index') is not None:
            starting_body_index = kwargs['starting_body_index']
        else:
            starting_body_index = random.randint(0, len(self.bodies)-1)
        starting_body = self.bodies[starting_body_index]
        starting_body_position = starting_body['body'].position

        # Add starting body to list of bodies in pattern
        pattern_bodies = []
        pattern_ptos = []
        pattern_bodies.append({"body": starting_body,
                               "original_body_index": starting_body_index,
                               "original_position": starting_body_position})

        # print("starting pos: ", pattern_bodies[0]['original_position'])

        # Create pattern
        if bodies_in_pattern > 1:
            for i in range(1, bodies_in_pattern):
                # Check to see if pattern already has all bodies in system
                if len(pattern_bodies) < len(self.bodies):
                    # Initialize list and variable for next body to add to pattern
                    possible_next_bodies = []
                    body_to_add = None
                    # Select which body in pattern will have a branch body added
                    branch_point_index = random.randint(0, len(pattern_bodies)-1)
                    branch_point = pattern_bodies[branch_point_index]

                    for body_index in range(len(self.bodies)):
                        body_found = False
                        if len(self.rotary_ptos) > 0:
                            for pto_index in range(0, len(self.rotary_ptos)):
                                temp_pto = self.rotary_ptos_data[pto_index]
                                if (temp_pto['idxa'] == branch_point['original_body_index']) \
                                        and (temp_pto['idxb'] == body_index):
                                    body_found = True
                                elif (temp_pto['idxb'] == branch_point['original_body_index']) \
                                        and (temp_pto['idxa'] == body_index):
                                    body_found = True
                                if body_found:
                                    possible_next_bodies.append((body_index, pto_index, "rotational"))
                                    break
                        if len(self.linear_ptos) > 0 and not body_found:
                            for pto_index in range(0, len(self.linear_ptos)):
                                temp_pto = self.linear_ptos_data[pto_index]
                                if (temp_pto['idxa'] == branch_point['original_body_index']) \
                                        and (temp_pto['idxb'] == body_index):
                                    body_found = True
                                elif (temp_pto['idxb'] == branch_point['original_body_index']) \
                                        and (temp_pto['idxa'] == body_index):
                                    body_found = True
                                if body_found:
                                    possible_next_bodies.append((body_index, pto_index, "linear"))
                                    break

                    # Select body from list of possible bodies to add to pattern
                    while len(possible_next_bodies) > 0:
                        is_in_pattern = False
                        i = random.randint(0, len(possible_next_bodies)-1)
                        check_body = possible_next_bodies[i]
                        check_body_original_index = check_body[0]
                        # Check to see if body is already in pattern
                        for j in range(0, len(pattern_bodies)):
                            if pattern_bodies[j]['original_body_index'] == check_body_original_index:
                                is_in_pattern = True
                                break
                        if not is_in_pattern:
                            body_to_add = check_body
                            break
                        else:
                            del possible_next_bodies[i]

                    if body_to_add is not None:
                        original_body_index = body_to_add[0]
                        original_pto_index = body_to_add[1]
                        pto_type = body_to_add[2]

                        original_body = self.bodies[original_body_index]
                        original_position = original_body['body'].position

                        if pto_type is 'rotational':
                            original_pto = self.rotary_ptos_data[original_pto_index]
                        elif pto_type is 'linear':
                            original_pto = self.linear_ptos_data[original_pto_index]

                        pattern_bodies.append({"body": original_body,
                                               "original_body_index": original_body_index,
                                               "original_position": original_position})
                        pattern_ptos.append({"pto": original_pto,
                                             "pto_type": pto_type})

        # Select which side of system to attach pattern
        attachment_side = random.randint(0, 1)
        if attachment_side == 0:
            attachment_side = 'left'
        elif attachment_side == 1:
            attachment_side = 'right'

        for current_pattern_iteration in range(0, num_patterns):
            # Find which body in system is farthest on attachment side
            attachment_body_index = 0
            for check_body_index in range(0, len(self.bodies)):
                check_body = self.bodies[check_body_index]
                check_body_pos = check_body['body'].position
                attachment_body = self.bodies[attachment_body_index]
                attachment_body_pos = attachment_body['body'].position
                if attachment_side is 'left':
                    if (check_body_pos[0] < attachment_body_pos[0]) or \
                            (check_body_pos[0] == attachment_body_pos[0] and check_body_pos[1] < attachment_body_pos[1]):
                        attachment_body_index = check_body_index
                elif attachment_side is 'right':
                    if (check_body_pos[0] > attachment_body_pos[0]) or \
                            (check_body_pos[0] == attachment_body_pos[0] and check_body_pos[1] < attachment_body_pos[1]):
                        attachment_body_index = check_body_index

            # Locate pattern body at side of pattern closest to attachment side
            # (i.e. right-most body if attachment is on left, and vice-versa)
            starting_pattern_body_index = 0
            for check_pattern_body_index in range(0, len(pattern_bodies)):
                check_pattern_body = pattern_bodies[check_pattern_body_index]
                check_pattern_body_pos = check_pattern_body['original_position']
                starting_body = pattern_bodies[starting_pattern_body_index]
                starting_body_pos = starting_body['original_position']
                if attachment_side is 'left':
                    if (check_pattern_body_pos[0] > starting_body_pos[0]) or \
                            (check_body_pos[0] == starting_body_pos[0] and check_body_pos[1] < starting_body_pos[1]):
                        starting_pattern_body_index = check_pattern_body_index
                elif attachment_side is 'right':
                    if (check_pattern_body_pos[0] < starting_body_pos[0]) or \
                            (check_body_pos[0] == starting_body_pos[0] and check_body_pos[1] < starting_body_pos[1]):
                        starting_pattern_body_index = check_pattern_body_index

            #
            starting_body = pattern_bodies[starting_pattern_body_index]
            starting_body_pos = starting_body['original_position']
            starting_body_rad = starting_body['body']['radius']
            attachment_body = self.bodies[attachment_body_index]
            attachment_body_pos = attachment_body['body'].position
            attachment_body_rad = attachment_body['radius']

            if attachment_side is 'left':
                side_shift = -(starting_body_rad + attachment_body_rad + self.pto_size)
            elif attachment_side is 'right':
                side_shift = (starting_body_rad + attachment_body_rad + self.pto_size)

            x_shift = attachment_body_pos[0] - starting_body_pos[0] + side_shift
            y_shift = attachment_body_pos[1] - starting_body_pos[1]

            for index in range(0, len(pattern_bodies)):
                temp_body = pattern_bodies[index]
                temp_body_data = temp_body['body']
                original_position = temp_body['original_position']
                shape = 'sphere'
                radius = temp_body_data['radius']

                new_x = original_position[0] + x_shift
                new_y = original_position[1] + y_shift

                if radius + new_y < self.sea_level:
                    density = 1000
                elif new_y - radius > self.sea_level:
                    density = 350
                else:
                    h = radius - new_y + self.sea_level
                    c = np.sqrt(h*(2*radius - h))
                    density = (self.rho_w * (np.pi / 6) * h * (3 * c * c + h * h))/ \
                              ((4.0 / 3.0) * np.pi * np.power(radius, 3))

                self.add_body(shape, density, (new_x, new_y),
                              radius=radius)

            self.add_rotational_pto(attachment_body_index,
                                    len(self.bodies)-len(pattern_bodies)+starting_pattern_body_index, 0, 10000, 1000000)
            if len(pattern_ptos) > 0:
                for index in range(0, len(pattern_ptos)):
                    temp_pto = pattern_ptos[index]
                    temp_pto_data = temp_pto['pto']
                    temp_pto_type = temp_pto['pto_type']

                    idxa = temp_pto_data['idxa']
                    idxb = temp_pto_data['idxb']

                    for j in range(0, len(pattern_bodies)):
                        if pattern_bodies[j]['original_body_index'] == idxa:
                            idxa = len(self.bodies) - len(pattern_bodies) + j
                        elif pattern_bodies[j]['original_body_index'] == idxb:
                            idxb = len(self.bodies) - len(pattern_bodies) + j

                    stiffness = temp_pto_data['stiffness']
                    damping = temp_pto_data['damping']

                    if temp_pto_type is 'rotational':
                        rest_angle = 0
                        self.add_rotational_pto(idxa, idxb, rest_angle, stiffness, damping)
                    elif temp_pto_type is 'linear':
                        pos_a = self.bodies[idxa]["body"].position
                        pos_b = self.bodies[idxb]["body"].position
                        resting_length = np.sqrt((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2) + self.pto_size
                        self.add_constrained_linear_pto(idxa, idxb, resting_length, stiffness, damping)

        # TODO: FIGURE OUT METHOD FOR ENSURE NO BODIES OUT OF WATER IN THIS CASE
        bodies_out_of_water = False
        for body in self.bodies:
            if body['body'].position[1] - body['radius'] >= self.sea_level:
                bodies_out_of_water = True
                break

        i = 0
        while bodies_out_of_water and i < 50:
            self.rule_check()
            for deletable in self.deletable_bodies:
                temp_body_check = self.bodies[deletable[0]]
                if temp_body_check['body'].position[1] - temp_body_check['radius'] >= self.sea_level:
                    self.remove_body_with_joint(deletable[0], deletable[1], deletable[2])
                    break
            bodies_out_of_water = False
            for body in self.bodies:
                if body['body'].position[1] - body['radius'] >= self.sea_level:
                    bodies_out_of_water = True
                    break
            i += 1

        self.applied_rules.append('H5')

    def standardize(self, **kwargs):
        standardize_bodies = False
        standardize_ptos = False

        # Check kwargs to see if specific input was given
        if kwargs.get('standardization_select') is not None:
            standardization_select = kwargs['standardization_select']
            if standardization_select is 'bodies' and len(self.bodies) > 1:
                standardize_bodies = True
            elif standardization_select is 'ptos' and (len(self.linear_ptos) + len(self.rotary_ptos) > 1):
                standardize_ptos = True
        else:
            standardization_select = random.randint(0, 1)
            if standardization_select == 0 and len(self.bodies) > 1:
                standardize_bodies = True
            elif standardization_select == 1 and (len(self.linear_ptos) + len(self.rotary_ptos) > 1):
                standardize_ptos = True

        # Standardiztion rate is based on a percentage
        min_standard = 25
        max_standard = 100

        # Check kwargs to see if specific input was given
        if kwargs.get('standardization_rate') is not None:
            standardization_rate = kwargs['standardization_rate']
        else:
            standardization_rate = random.randint(min_standard, max_standard)/100

        # print('Standardization rate: ', standardization_rate)

        if standardize_bodies:
            # Possible options: density, dimensions
            change_density = False
            change_radius = False

            # Check kwargs to see if specific input was given
            if kwargs.get('base_index') is not None:
                base_index = kwargs['base_index']
            else:
                base_index = random.randint(0, len(self.bodies)-1)
            base = self.bodies[base_index]
            base_density = base['density']
            base_radius = base['radius']

            # Check kwargs to see if specific input was given
            if kwargs.get('num_to_standardize') is not None:
                num_to_standardize = kwargs['num_to_standardize']
            else:
                num_to_standardize = random.randint(1, len(self.bodies)-1)
            bodies_to_standardize = []
            for num in range(0, num_to_standardize):
                while True:
                    index = random.randint(0, len(self.bodies)-1)
                    if index not in bodies_to_standardize and index != base_index:
                        bodies_to_standardize.append(index)
                        break
            # print("Base index: ", base_index)
            # print("Base radius: ", base_radius)
            # print("Base density: ", base_density)
            # print("Bodies to standardize: ", bodies_to_standardize)

            for i in range(0, num_to_standardize):
                body_index = bodies_to_standardize[i]
                body = self.bodies[body_index]
                # body_density = body['density']
                body_radius = body['radius']
                body_length = body['length']
                body_angle_offset = body['angle_offset']

                # density_diff = base_density - body_density
                radius_diff = base_radius - body_radius

                # new_density = body_density + density_diff*standardization_rate
                new_radius = int(body_radius + radius_diff*standardization_rate)

                # self.change_body_density(body_index, new_density)
                self.change_body_dimensions(body_index,
                                            radius=new_radius, length=body_length, angle_offset=body_angle_offset)
                # print("Body: ", body_index, ", old radius: ", body_radius, ", new radius: ", new_radius)
                # print("Body: ", body_index, ", old density: ", body_density, ", new density: ", new_density)
        elif standardize_ptos:
            # Possible values to change: type, stiffness, damping
            while True:
                # Check kwargs to see if specific input was given
                if kwargs.get('type_selection') is not None:
                    type_selection = kwargs['type_selection']
                    if type_selection is 'rotational':
                        type_selection = 0
                    elif type_selection is 'linear:':
                        type_selection = 1
                else:
                    type_selection = random.randint(0, 1)

                if type_selection == 0 and len(self.rotary_ptos) > 0:
                    base_type = 'rotational'
                    base_index = random.randint(0, len(self.rotary_ptos)-1)
                    base_pto = self.rotary_ptos_data[base_index]
                    base_stiffness = base_pto['stiffness']
                    base_damping = base_pto['damping']
                    break
                elif type_selection == 1 and len(self.linear_ptos) > 0:
                    base_type = 'linear'
                    base_index = random.randint(0, len(self.linear_ptos)-1)
                    base_pto = self.linear_ptos_data[base_index]
                    base_stiffness = base_pto['stiffness']
                    base_damping = base_pto['damping']
                    break

            # print('Type: ', base_type)
            # print('Base index: ', base_index)
            # print('Base pto: ', base_pto)
            # print('Base stiffness: ', base_stiffness)
            # print('Base damping: ', base_damping)

            num_to_standardize = random.randint(1, len(self.linear_ptos)+len(self.rotary_ptos)-1)
            ptos_to_standardize = []

            # print('number to standardize: ', num_to_standardize)
            for num in range(0, num_to_standardize):
                while True:
                    pto_valid = False
                    guess_type_selection = random.randint(0, 1)
                    if guess_type_selection == 0 and len(self.rotary_ptos) > 0:
                        guess_pto_index = random.randint(0, len(self.rotary_ptos)-1)
                        guess_pto = ({'pto_index': guess_pto_index,
                                      'pto_type': 'rotational'})
                        pto_valid = True
                    elif guess_type_selection == 1 and len(self.linear_ptos) > 0:
                        guess_pto_index = random.randint(0, len(self.linear_ptos)-1)
                        guess_pto = ({'pto_index': guess_pto_index,
                                      'pto_type': 'linear'})
                        pto_valid = True
                    for saved_pto in ptos_to_standardize:
                        if guess_pto['pto_type'] is not saved_pto['pto_type'] or \
                                (guess_pto['pto_type'] is saved_pto['pto_type'] and
                                    guess_pto['pto_index'] is not saved_pto['pto_index']):
                            pto_valid = True
                        else:
                            pto_valid = False
                            break
                    if pto_valid:
                        ptos_to_standardize.append(guess_pto)
                        break
            # print('ptos to standardize: ', ptos_to_standardize)

            for i in range(0, num_to_standardize):
                pto_index = ptos_to_standardize[i]['pto_index']
                pto_type = ptos_to_standardize[i]['pto_type']
                if pto_type is 'rotational':
                    pto = self.rotary_ptos_data[pto_index]
                    pto_stiffness = pto['stiffness']
                    pto_damping = pto['damping']

                elif pto_type is 'linear':
                    pto = self.linear_ptos_data[pto_index]
                    pto_stiffness = pto['stiffness']
                    pto_damping = pto['damping']

                stiffness_diff = base_stiffness - pto_stiffness
                damping_diff = base_damping - pto_damping

                new_stiffness = pto_stiffness + stiffness_diff*standardization_rate
                new_damping = pto_damping + damping_diff*standardization_rate

                self.lowtier_rule_perform(8, joint_index=pto_index, joint_type=pto_type,
                                          stiffness=new_stiffness, damping=new_damping)
                # Remove the last addition to list of applied rules to stop list from showing low-tier rule applied
                del self.applied_rules[-1]

                # Convert joint type if joint is not same as 'standard' joint
                if base_type is not pto_type:
                    self.change_joint_type(pto_index, pto_type)

                    # Check to see if conversion of pto type requires index of next standardized joints to be changed
                    for check_index in range(i, num_to_standardize):
                        check_pto = ptos_to_standardize[check_index]
                        if check_pto['pto_type'] is pto_type and check_pto['pto_index'] > pto_index:
                            ptos_to_standardize[check_index]['pto_index'] -= 1
        self.applied_rules.append('H6')

    # End of higher-tier operations

    def add_buoyant_force(self):
        for body in self.bodies:
            # TODO Add general computation that is voxel-based
            # if body["body_shape"] is "sphere":
            #     r = body["radius"]
            #     if r + body["body"].position[1] < self.sea_level:
            #         displaced_mass = body["volume"] * self.rho_w
            #     elif body["body"].position[1] - r > self.sea_level:
            #         displaced_mass = 0
            #     else:
            #         h = r - body["body"].position[1] + self.sea_level
            #         c = np.sqrt(h * (2 * r - h))
            #         displaced_mass = self.rho_w * (np.pi / 6) * h * (3 * c * c + h * h)
            # elif body["body_shape"] is "cylinder":
            #     r = body["radius"]
            #     l = body["length"]
            #     index = self.world.bodies.index(body["body"])
            #     angle = self.world.bodies[index].angle
            #     theta = (angle*np.pi)/180
            #     phi = np.arctan(r/l)
            #     d = np.sqrt((l/2)**2 + (r/2)**2) * np.sin(theta + phi)
            #     if d + body["body"].position[1] < self.sea_level:
            #         displaced_mass = body["volume"] * self.rho_w
            #     elif body["body"].position[1] - d > self.sea_level:
            #         displaced_mass = 0
            #     else:
            #         h = d - body["body"].position[1] + self.sea_level
            #         beta = np.pi/2 - theta
            #         x1 = h/np.sin(beta)
            #         x2 = h/np.sin(theta)
            #         A = 0.5 * x1 * x2
            #         volume = A * r
            #         print(theta)
            #         print(beta)
            #         print(" ")
            #         # displaced_mass = volume * self.rho_w
            #         displaced_mass = 0
            r = body["radius"]
            if r + body["body"].position[1] < self.sea_level:
                displaced_mass = body["volume"] * self.rho_w
            elif body["body"].position[1] - r > self.sea_level:
                displaced_mass = 0
            else:
                h = r - body["body"].position[1] + self.sea_level
                c = np.sqrt(h * (2 * r - h))
                displaced_mass = self.rho_w * (np.pi / 6) * h * (3 * c * c + h * h)

            faddl = -displaced_mass * self.world.gravity
            body["body"].apply_force_at_world_point(faddl, body["body"].local_to_world(body["body"].center_of_gravity))

    def voxelize_bodies(self):
        for body in self.bodies:
            if body["voxels"] is []:
                if body["shape"] is "sphere":
                    body["voxels"] = []
                elif body["shape"] is "cylinder":
                    body["voxels"] = []

    def add_excitation_force(self):
        for body in self.bodies:
            if body['body'].position[1] >= self.sea_level - body['radius']:
                # amplitude = 42346*body["radius"]*2
                amplitude = 42346 * body["radius"] * 2 / 2
                body["body"].apply_force_at_world_point(
                    (0, np.sqrt(2*0.01)*amplitude * np.sin((self.iter / 1000)*2*np.pi - 2*np.pi*body['body'].position[0]/100)),
                    body["body"].local_to_world(body["body"].center_of_gravity))

    def add_radiative_force(self):
        for body in self.bodies:
            body["body"].apply_force_at_world_point((0, 0), body["body"].local_to_world(body["body"].center_of_gravity))

    def add_viscous_force(self):
        for body in self.bodies:
            # Compute viscous drag of sphere perpendicular to force,
            Cd = 0.47
            v = body["body"].velocity
            A = np.pi * np.power(body["radius"], 2)
            f = -0.5 * self.rho_w * Cd * A * v * np.linalg.norm(v)

            body["body"].apply_force_at_world_point(f, body["body"].local_to_world(body["body"].center_of_gravity))

    def pull_position_data(self):
        for i in range(len(self.bodies)):
            self.bodies[i]['xyz'][self.iter, 0] = self.bodies[i]['body'].position[0]
            self.bodies[i]['xyz'][self.iter, 1] = self.bodies[i]['body'].position[1]

    def evaluate(self):
        self.stable_system = True
        ss_count = 0
        energy = np.zeros(2)
        if self.display_visual:
            self.display = wec.wec_visual.wec_visual()

        while self.iter < self.simulation_steps + self.initial_steps:

            if self.display_visual:
                self.display.display(self)

            # Add forces
            self.add_buoyant_force()
            # Add wave force once steady-state has been reached
            if self.iter > self.initial_steps:
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
                if np.linalg.norm(body['last_velocity']) > 50:
                    self.stable_system = False
                    print("ITERATION UNSTABLE: BODY VELOCITY EXCEEDS MAXIMUM")
                    print('')
                    break

            # Check for steady state
            if self.iter < self.initial_steps:
                steady_state_reached = False
                temp_steady_state = True
                for body in self.bodies:
                    velocity = np.linalg.norm(body['last_velocity'])
                    # print(velocity)
                    if velocity >= 0.25:
                        temp_steady_state = False
                        break
                if temp_steady_state:
                    ss_count += 1
                else:
                    ss_count = 0
                if ss_count > 500:
                    steady_state_reached = True
                if steady_state_reached:
                    self.iter = self.initial_steps
                    print("ss reached")
                    print("")

            # Pull position data
            self.pull_position_data()

            # Track PTO energy extraction after reaching steady-state
            if self.iter >= self.initial_steps:
                if not steady_state_reached:
                    self.stable_system = False
                    print("ITERATION UNSTABLE: STEADY-STATE NOT REACHED")
                    print('')
                for pto in self.linear_ptos:
                    relative_velocity = np.linalg.norm(pto.a.velocity - pto.b.velocity)
                    energy[0] += np.power(relative_velocity, 2) * pto.damping * self.simulation_dt
                for pto in self.rotary_ptos:
                    relative_velocity = np.abs(pto.a.angular_velocity - pto.b.angular_velocity)
                    energy[1] += np.power(relative_velocity, 2) * pto.damping * self.simulation_dt

            if not self.stable_system:
                break

            self.iter += 1

            if self.display_visual:
                time.sleep(.0000001)

        self.iter = 0

        # Calculate power and add together to final total power produced
        self.power = sum(energy / (self.simulation_dt * self.simulation_steps))
        # If power is zero, set power to arbitrarily small value so comparisons do not divide by zero in agent
        if self.power == 0:
            self.power = 1e-40

        # Calculate mass
        self.mass = 0
        for body in self.bodies:
            self.mass += body['mass']

        # Calculate number of pieces
        self.pieces = len(self.linear_ptos) + len(self.rotary_ptos) + len(self.bodies)

        return [self.mass, self.power, self.pieces]

    # Low-tier random rule selection
    def lowtier_rule_select(self):
        num_rules = 8
        rule = random.randint(1, num_rules)

        # TODO: THIS IS ONLY FOR THE TESTS WHICH OPTIMIZE PRE-DETERMINED TOPOLOGIES
        # num_rules = 2
        # rule = random.randint(0, num_rules-1)
        # if rule == 0:
        #     rule = 5
        # elif rule == 1:
        #     rule = 8

        print("LT Rule:", rule)
        return rule

    # High-tier random rule selection
    def hightier_rule_select(self):
        # Not counting H1 (Generate initial design) as option for rule application
        num_rules = 6
        rule = random.randint(2, num_rules)
        print("HT Rule:", rule)
        return rule

    # Perform low-tier rule. Can select randomized values or take specific inputs for each rule
    def lowtier_rule_perform(self, rule, **kwargs):
        y_min = 50
        y_max = self.sea_level
        x_min = -100
        x_max = 100
        radius_min = 1
        radius_max = 5
        length_min = 1
        length_max = 100
        angle_min = 0
        angle_max = 90
        density_min = 600
        density_max = 1200
        rest_angle = 0
        stiffness_min = 5000
        stiffness_max = 15000
        damping_min = 500000
        damping_max = 1500000
        mooring_depth = 50

        valid_rule = False

        # Add rotational body
        if rule == 1:
            shape_select = random.randint(0, 0)
            if shape_select == 0:
                shape = 'sphere'
            else:
                shape = 'cylinder'
            if kwargs.get('radius') is not None:
                radius = kwargs['radius']
            else:
                radius = random.randint(radius_min*10, radius_max*10)/10
            if kwargs.get('length') is not None:
                length = kwargs['length']
            else:
                length = random.randint(length_min, length_max)
            if kwargs.get('angle') is not None:
                angle = kwargs['angle']
            else:
                angle = random.randint(angle_min, angle_max)
            if kwargs.get('stiffness') is not None:
                stiffness = kwargs['stiffness']
            else:
                stiffness = random.randint(stiffness_min, stiffness_max)
            if kwargs.get('damping') is not None:
                damping = kwargs['damping']
            else:
                damping = random.randint(damping_min, damping_max)

            self.rule_check()
            if kwargs.get('location') is not None:
                location = kwargs['location']
            else:
                location = self.addition_locations[random.randint(0, len(self.addition_locations) - 1)]
            body = location[0]
            body_pos = self.bodies[body]['body'].position
            body_radius = self.bodies[body]['radius']
            if location[1] == 0:
                x = body_pos[0] + body_radius + radius + self.pto_size
                y = body_pos[1]
            elif location[1] == 1:
                x = body_pos[0]
                y = body_pos[1] + body_radius + radius + self.pto_size
            elif location[1] == 2:
                x = body_pos[0] - body_radius - radius - self.pto_size
                y = body_pos[1]
            elif location[1] == 3:
                x = body_pos[0]
                y = body_pos[1] - body_radius - radius - self.pto_size

            # If not specified density, select density to keep system in steady-state condition
            if kwargs.get('density') is not None:
                density = kwargs['density']
            else:
                if radius + y < self.sea_level:
                    density = 1000
                elif y - radius > self.sea_level:
                    density = 350
                else:
                    h = radius - y + self.sea_level
                    c = np.sqrt(h*(2*radius - h))
                    density = (self.rho_w * (np.pi / 6) * h * (3 * c * c + h * h))/ \
                              ((4.0 / 3.0) * np.pi * np.power(radius, 3))

            self.add_rotational_body(shape, density, (x, y), body, rest_angle, stiffness, damping,
                                     radius=radius, length=length, angle_offset=angle)
            self.applied_rules.append('L1')

        # Add linear body
        elif rule == 2:
            shape_select = random.randint(0, 0)
            if shape_select == 0:
                shape = 'sphere'
            else:
                shape = 'cylinder'
            if kwargs.get('radius') is not None:
                radius = kwargs['radius']
            else:
                radius = random.randint(radius_min*10, radius_max*10)/10
            if kwargs.get('length') is not None:
                length = kwargs['length']
            else:
                length = random.randint(length_min, length_max)
            if kwargs.get('angle') is not None:
                angle = kwargs['angle']
            else:
                angle = random.randint(angle_min, angle_max)
            if kwargs.get('stiffness') is not None:
                stiffness = kwargs['stiffness']
            else:
                stiffness = random.randint(stiffness_min, stiffness_max)
            if kwargs.get('damping') is not None:
                damping = kwargs['damping']
            else:
                damping = random.randint(damping_min, damping_max)

            self.rule_check()
            if kwargs.get('location') is not None:
                location = kwargs['location']
            else:
                location = self.addition_locations[random.randint(0, len(self.addition_locations) - 1)]
            body = location[0]
            body_pos = self.bodies[body]['body'].position
            body_radius = self.bodies[body]['radius']
            if location[1] == 0:
                x = body_pos[0] + body_radius + radius + self.pto_size
                y = body_pos[1]
            elif location[1] == 1:
                x = body_pos[0]
                y = body_pos[1] + body_radius + radius + self.pto_size
            elif location[1] == 2:
                x = body_pos[0] - body_radius - radius - self.pto_size
                y = body_pos[1]
            elif location[1] == 3:
                x = body_pos[0]
                y = body_pos[1] - body_radius - radius - self.pto_size

            # If not specified density, select density to keep system in steady-state condition
            if kwargs.get('density') is not None:
                density = kwargs['density']
            else:
                if radius + y < self.sea_level:
                    density = 1000
                elif y - radius > self.sea_level:
                    density = 350
                else:
                    h = radius - y + self.sea_level
                    c = np.sqrt(h * (2*radius - h))
                    density = (self.rho_w * (np.pi / 6) * h * (3 * c * c + h * h)) / \
                              ((4.0 / 3.0) * np.pi * np.power(radius, 3))

            self.add_linear_body(shape, density, (x, y), body, stiffness, damping,
                                 radius=radius, length=length, angle_offset=angle)
            self.applied_rules.append('L2')

        # Delete body with joint
        elif rule == 3:
            if len(self.bodies) > 1:
                self.rule_check()
                if kwargs.get('removal') is not None:
                    removal = kwargs['removal']
                else:
                    removal = self.deletable_bodies[random.randint(0, len(self.deletable_bodies)-1)]
                self.remove_body_with_joint(removal[0], removal[1], removal[2])
                self.applied_rules.append('L3')

        # Change joint type
        elif rule == 4:
            if len(self.rotary_ptos) > 0 or len(self.linear_ptos) > 0:
                while not valid_rule:
                    type_select = random.randint(0, 1)
                    if type_select == 0 and len(self.rotary_ptos) > 0:
                        pto = random.randint(0, len(self.rotary_ptos)-1)
                        self.change_joint_type(pto, 'rotational')
                        valid_rule = True
                    elif type_select == 1 and len(self.linear_ptos) > 0:
                        pto = random.randint(0, len(self.linear_ptos)-1)
                        self.change_joint_type(pto, 'linear')
                        valid_rule = True
                self.applied_rules.append('L4')

        # Change body dimensions
        elif rule == 5:
            if len(self.bodies) > 0:
                if kwargs.get('body') is not None:
                    body = kwargs['body']
                else:
                    body = random.randint(0, len(self.bodies)-1)
                if kwargs.get('radius') is not None:
                    radius = kwargs['radius']
                else:
                    radius = random.randint(radius_min, radius_max)
                if kwargs.get('length') is not None:
                    length = kwargs['length']
                else:
                    length = random.randint(length_min, length_max)
                if kwargs.get('angle') is not None:
                    angle = kwargs['angle']
                else:
                    angle = random.randint(angle_min, angle_max)
                self.change_body_dimensions(body, radius=radius, length=length, angle=angle)
                self.applied_rules.append('L5')

        # # Change body density
        # elif rule == 6:
        #     if len(self.bodies) > 0:
        #         if kwargs.get('body') is not None:
        #             body = kwargs['body']
        #         else:
        #             body = random.randint(0, len(self.bodies)-1)
        #         if kwargs.get('density') is not None:
        #             density = kwargs['density']
        #         else:
        #             density = random.randint(density_min, density_max)
        #         self.change_body_density(body, density)
        #         self.applied_rules.append('L6')

        # Relocate body with joint
        elif rule == 6:
            if len(self.bodies) > 1:
                self.rule_check()
                if kwargs.get('removal') is not None:
                    removal = kwargs['removal']
                else:
                    removal = self.deletable_bodies[random.randint(0, len(self.deletable_bodies) - 1)]
                if kwargs.get('location') is not None:
                    location = kwargs['location']
                    attachment_body = location[0]
                else:
                    while True:
                        location = self.addition_locations[random.randint(0, len(self.addition_locations) - 1)]
                        attachment_body = location[0]
                        if attachment_body != removal[0]:
                            break

                attachment_body_pos = self.bodies[attachment_body]['body'].position
                attachment_body_radius = self.bodies[attachment_body]['radius']
                radius = self.bodies[removal[0]]['radius']
                if location[1] == 0:
                    x = attachment_body_pos[0] + attachment_body_radius + radius + self.pto_size
                    y = attachment_body_pos[1]
                elif location[1] == 1:
                    x = attachment_body_pos[0]
                    y = attachment_body_pos[1] + attachment_body_radius + radius + self.pto_size
                elif location[1] == 2:
                    x = attachment_body_pos[0] - attachment_body_radius - radius - self.pto_size
                    y = attachment_body_pos[1]
                elif location[1] == 3:
                    x = attachment_body_pos[0]
                    y = attachment_body_pos[1] - attachment_body_radius - radius- self.pto_size
                self.relocate_body_with_joint(removal[0], removal[1], removal[2], (x, y), attachment_body)
                self.applied_rules.append('L6')

        # Swap bodies
        elif rule == 7:
            if len(self.bodies) > 1:
                if kwargs.get('body_a') is not None:
                    body_a = kwargs['body_a']
                else:
                    body_a = random.randint(0, len(self.bodies)-1)
                if kwargs.get('body_b') is not None:
                    body_b = kwargs['body_b']
                else:
                    while True:
                        body_b = random.randint(0, len(self.bodies)-1)
                        if body_a != body_b:
                            break
                self.swap_bodies(body_a, body_b)
                self.applied_rules.append('L7')

        # Change joint coefficients
        elif rule == 8:
            if len(self.rotary_ptos) > 0 or len(self.linear_ptos) > 0:
                if kwargs.get('joint_type') is not None:
                    joint_type = kwargs['joint_type']
                else:
                    while not valid_rule:
                        type_select = random.randint(0, 1)
                        if type_select == 0 and len(self.rotary_ptos) > 0:
                            joint_type = 'rotational'
                            valid_rule = True
                        elif type_select == 1 and len(self.linear_ptos) > 0:
                            joint_type = 'linear'
                            valid_rule = True
                if kwargs.get('joint_index') is not None:
                    joint_index = kwargs['joint_index']
                else:
                    if joint_type is 'rotational':
                        joint_index = random.randint(0, len(self.rotary_ptos)-1)
                    elif joint_type is 'linear':
                        joint_index = random.randint(0, len(self.linear_ptos)-1)
                if kwargs.get('stiffness') is not None:
                    stiffness = kwargs['stiffness']
                else:
                    stiffness = random.randint(stiffness_min, stiffness_max)
                if kwargs.get('damping') is not None:
                    damping = kwargs['damping']
                else:
                    damping = random.randint(damping_min, damping_max)
                self.change_joint_coefficients(joint_index, joint_type, stiffness=stiffness, damping=damping)
                self.applied_rules.append('L8')

        # Add mooring system
        elif rule == 10:
            if len(self.bodies) > 0:
                x = random.randint(x_min, x_max)
                y = mooring_depth
                body = random.randint(0, len(self.bodies)-1)
                stiffness = random.randint(stiffness_min, stiffness_max)
                damping = random.randint(damping_min, damping_max)
                self.add_mooring_system((x, y), body, stiffness, damping)

        # Delete mooring system
        elif rule == 11:
            if len(self.mooring_attachment_points) > 0:
                mooring = random.randint(0, len(self.mooring_attachment_points)-1)
                self.remove_mooring_system(mooring)

        # Relocate cable attachment point
        elif rule == 12:
            if len(self.mooring_attachment_points) > 0:
                mooring = random.randint(0, len(self.mooring_attachment_points)-1)
                body = random.randint(0, len(self.bodies)-1)
                self.relocate_mooring_cable_attachment(mooring, body)

        # Relocate fixed body
        elif rule == 13:
            # print("Attachment points: ", self.mooring_attachment_points)
            # print("Fixed bodies: ", self.fixed_bodies)
            # print("Cable bodies: ", self.cable_bodies)
            if len(self.mooring_attachment_points) > 0:
                x = random.randint(x_min, x_max)
                y = mooring_depth
                mooring = random.randint(0, len(self.mooring_attachment_points)-1)
                self.relocate_mooring_fixed_body(mooring, (x, y))

    # Perform high-tier rule. Can select randomized values or take specific inputs for each rule
    def hightier_rule_perform(self, rule, **kwargs):
        # Create initial design
        if rule == 1:
            self.create_initial_design(**kwargs)
        # Increase complexity
        elif rule == 2:
            self.increase_or_decrease_complexity(complexity_change_type='increase', **kwargs)
        # Decrease complexity
        elif rule == 3:
            self.increase_or_decrease_complexity(complexity_change_type='decrease', **kwargs)
        # Change design scale
        elif rule == 4:
            self.change_design_scale(**kwargs)
        # Replicate pattern
        elif rule == 5:
            self.replicate_pattern(**kwargs)
        # Standardize
        elif rule == 6:
            self.standardize(**kwargs)

    # Check to see if any bodies are overlapping
    def is_valid(self):
        validity = True
        for body_a in self.world.shapes:
            for body_b in self.world.shapes:
                collision = body_a.shapes_collide(body_b)
                if (collision.normal[0] != 0 or collision.normal[1] != 0) and body_a != body_b:
                    validity = False
                    break
                else:
                    validity = True
            if not validity:
                break
        print("Solution Valid?:", validity)
        print("")
        return validity

    # Search through design to check where bodies can be added and where bodies can be deleted
    def rule_check(self):
        self.addition_locations = []
        self.deletable_bodies = []

        for pos_to_check in range(0, 4):
            for i in range(0, len(self.bodies)):
                body = self.bodies[i]
                body_pos = body['body'].position
                body_radius = body['radius']
                for j in range(0, len(self.bodies)):
                    test_body = self.bodies[j]
                    test_body_pos = test_body['body'].position
                    test_body_radius = test_body['radius']
                    distance = np.sqrt((body_pos[0]-test_body_pos[0])**2 + (body_pos[1]-test_body_pos[1])**2)
                    if distance <= body_radius + test_body_radius + self.pto_size + 0.05 and \
                                    distance >= body_radius + test_body_radius + self.pto_size - 0.05:
                        if pos_to_check == 0:
                            if test_body_pos[0] + 0.05 < body_pos[0] + distance:
                                valid_location = True
                            else:
                                valid_location = False
                                break
                        elif pos_to_check == 1:
                            if test_body_pos[1] + 0.05 < body_pos[1] + distance and body_pos[1] + body_radius < self.sea_level:
                                valid_location = True
                            else:
                                valid_location = False
                                break
                        elif pos_to_check == 2:
                            if test_body_pos[0] - 0.05 > body_pos[0] - distance:
                                valid_location = True
                            else:
                                valid_location = False
                                break
                        elif pos_to_check == 3:
                            if test_body_pos[1] - 0.05 > body_pos[1] - distance:
                                valid_location = True
                            else:
                                valid_location = False
                                break
                    else:
                        valid_location = True
                    if not valid_location:
                        break
                if valid_location:
                    self.addition_locations.append((i, pos_to_check))

        for n in range(0, len(self.bodies)):
            connections = 0
            i = 0
            for pto in self.rotary_ptos_data:
                if pto['idxa'] == n or pto['idxb'] == n:
                    connections += 1
                    pto_index = i
                    type = 'rotational'
                i += 1
            i = 0
            for pto in self.linear_ptos_data:
                if pto['idxa'] == n or pto['idxb'] == n:
                    connections += 1
                    pto_index = i
                    type = 'linear'
                i += 1
            if connections == 1:
                self.deletable_bodies.append((n, pto_index, type))

    # Function needed to make copy of WEC object by agent
    def __deepcopy__(self, memo):
        deepcopy_method = self.__deepcopy__
        self.__deepcopy__ = None
        cp = copy.deepcopy(self, memo)
        self.__deepcopy__ = deepcopy_method
        return cp