import numpy as np
import pymunk as pm

from heuristic_bursts.abstract_base_solution import AbstractBaseSolution

import wec.spectrum as sp
import wec.excitation_forces as ef
import pkg_resources
import random

import wec.wec_visual
import time

class WEC(AbstractBaseSolution):
    simulation_dt = 0.1
    simulation_steps = 1000
    initial_steps = 10000
    error_bias = 0.02
    spectrum = sp.Spectrum('bretschneider', fp=0.5, Hm0=2)
    forces = ef.ExcitationForces()
    forces.load_model(pkg_resources.resource_filename('wec', 'data/full_network_structure.yml'),
                      pkg_resources.resource_filename('wec', 'data/full_network_weights.h5'))
    rho_w = 1000
    gravity = -9.81
    number_of_metrics = 3
    sea_level = 400
    pto_size = 1

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
        self.addition_locations = []
        self.deletable_bodies = []

        # Initialize lists for mooring system
        self.fixed_bodies = []
        self.cable_bodies = []
        self.mooring_attachment_points = []

        # current iteration
        self.iter = 0
        self.display_visual = False

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
        resting_length = np.sqrt((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2) + self.pto_size
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
        for relative_position in range(0, 4):
            self.reposition_branch(index, relative_position, initial_body, new_body)

    def reposition_branch(self, body_index, relative_position, initial_body, new_body):
        old_pos = initial_body['body'].position
        R1 = initial_body['radius']
        R2 = new_body['radius']
        rad_diff = R2 - R1

        bodies_to_check = []
        bodies_to_shift = []

        for i in range(0, len(self.bodies)):
            test_body = self.bodies[i]
            test_body_pos = test_body['body'].position
            test_body_radius = test_body['radius']
            distance = np.sqrt((old_pos[0] - test_body_pos[0]) ** 2 + (old_pos[1] - test_body_pos[1]) ** 2)
            if distance == R1 + test_body_radius + self.pto_size:
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
                temp_body_pos[0] += rad_diff
            elif relative_position == 1:
                temp_body_pos[1] += rad_diff
            elif relative_position == 2:
                temp_body_pos[0] -= rad_diff
            elif relative_position == 3:
                temp_body_pos[1] -= rad_diff

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
        density = temp_body["density"]
        radius = temp_body["radius"]
        length = temp_body["length"]
        angle_offset = temp_body["angle_offset"]

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
        initial_body_a = temp_body_a
        initial_body_b = temp_body_b

        self.remove_body(idxa)
        self.add_body(temp_body_b['body_shape'], temp_body_b['density'], temp_body_a['body'].position,
                      radius=temp_body_b['radius'], length=temp_body_b['length'], angle_offset=temp_body_b['angle_offset'])
        temp_body = self.bodies[-1]
        new_body_a = temp_body
        self.bodies.insert(idxa, temp_body)
        del self.bodies[-1]

        self.remove_body(idxb)
        self.add_body(temp_body_a['body_shape'], temp_body_a['density'], temp_body_b['body'].position,
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

    # LUCAS: Higher tier operations will go here eventually too. These should follow the function definitions as defined
    #        in the abstract base solution class.

    def create_initial_design(self, num_rules):
        for i in range(1, num_rules):
            rule = self.rule_select()
            self.rule_perform(rule)
            print('Validity: ', self.is_valid())

    def change_design_scale(self):
        min_multiplier = 0.5
        max_multiplier = 2.0

        valid_multiplier = False
        while not valid_multiplier:
            multiplier = random.uniform(min_multiplier, max_multiplier)
            for index in range(0, len(self.bodies)):
                if int(self.bodies[index]['radius'] * multiplier) < 1:
                    valid_multiplier = False
                    break
                else:
                    valid_multiplier = True

        for index in range(0, len(self.bodies)):
            body = self.bodies[index]
            radius = int(body['radius'] * multiplier)
            length = int(body['length'] * multiplier)
            angle_offset = int(body['angle_offset'] * multiplier)
            self.change_body_dimensions(index, radius=radius, length=length, angle_offset=angle_offset)

    def extend_or_reduce(self):
        extend = False
        reduce = False
        decision = random.randint(0, 1)
        if decision == 0:
            extend = True
        else:
            reduce = True

        min_length = 1
        max_length = 3
        rule_length = random.randint(min_length, max_length)

        self.rule_check()
        if extend:
            next_location = self.addition_locations[random.randint(0, len(self.addition_locations) - 1)]
            for n in range(0, rule_length):
                body_type = random.randint(0, 1)
                if body_type == 0:
                    self.rule_perform(1, location=next_location)
                elif body_type == 1:
                    self.rule_perform(2, location=next_location)
                self.rule_check()
                added_body_index = len(self.bodies) - 1
                while next_location[0] is not added_body_index:
                    next_location = self.addition_locations[random.randint(0, len(self.addition_locations) - 1)]

        elif reduce and len(self.deletable_bodies) > 0:
            next_delete = self.deletable_bodies[random.randint(0, len(self.deletable_bodies) - 1)]
            for n in range(0, rule_length):
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

                    self.rule_perform(3, removal=next_delete)

                    if temp_next_delete > next_delete[0]:
                        temp_next_delete -= 1

                    if len(self.bodies) > 1:
                        for body in self.deletable_bodies:
                            if body[0] == temp_next_delete:
                                continue_rule = True
                                break
                            else:
                                continue_rule = False
                    else:
                        continue_rule = False

                    if continue_rule:
                        while next_delete[0] is not temp_next_delete:
                            next_delete = self.deletable_bodies[random.randint(0, len(self.deletable_bodies) - 1)]
                    else:
                        break

    def increase_symmetry(self):
        pass

    def replicate_pattern(self):
        max_bodies_in_pattern = 4
        max_num_patterns = 3
        bodies_in_pattern = random.randint(1, min(max_bodies_in_pattern, len(self.bodies)))
        num_patterns = random.randint(1, max_num_patterns)

        starting_body = random.randint(0, len(self.bodies)-1)


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
            if body['body'].position[1] > self.sea_level - 10:
                body["body"].apply_force_at_world_point(
                    (0, body['mass'] * np.sin(self.iter / 100 + body['body'].position[0])),
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
        energy = np.zeros(2)
        if self.display_visual:
            self.display = wec.wec_visual.wec_visual()

        while self.iter < self.simulation_steps + self.initial_steps:

            if self.display_visual:
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

            # Track PTO energy extraction after reaching steady-state
            if self.iter > self.initial_steps:
                for pto in self.linear_ptos:
                    relative_velocity = np.linalg.norm(pto.a.velocity - pto.b.velocity)
                    energy[0] += np.power(relative_velocity, 2) * pto.damping * self.simulation_dt
                for pto in self.rotary_ptos:
                    relative_velocity = np.abs(pto.a.angular_velocity - pto.b.angular_velocity)
                    energy[1] += np.power(relative_velocity, 2) * pto.damping * self.simulation_dt

            self.iter += 1

            if self.display_visual:
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

    def rule_select(self):
        num_rules = 12
        rule = random.randint(1, num_rules)
        print(rule)
        return rule

    def rule_perform(self, rule, **kwargs):
        y_min = 50
        y_max = self.sea_level
        x_min = 0
        x_max = 800
        radius_min = 10
        radius_max = 25
        length_min = 1
        length_max = 100
        angle_min = 0
        angle_max = 90
        density_min = 500
        density_max = 1200
        rest_angle = 0
        stiffness_min = 100
        stiffness_max = 10000
        damping_min = 1000000
        damping_max = 100000000
        mooring_depth = 50

        valid_rule = False

        # Add rotational body
        if rule == 1:
            shape_select = random.randint(0, 0)
            if shape_select == 0:
                shape = 'sphere'
            else:
                shape = 'cylinder'
            density = random.randint(density_min, density_max)
            radius = random.randint(radius_min, radius_max)
            length = random.randint(length_min, length_max)
            angle = random.randint(angle_min, angle_max)
            stiffness = random.randint(stiffness_min, stiffness_max)
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

            self.add_rotational_body(shape, density, (x, y), body, rest_angle, stiffness, damping,
                                     radius=radius, length=length, angle_offset=angle)

        # Add linear body
        elif rule == 2:
            shape_select = random.randint(0, 0)
            if shape_select == 0:
                shape = 'sphere'
            else:
                shape = 'cylinder'
            density = random.randint(density_min, density_max)
            radius = random.randint(radius_min, radius_max)
            length = random.randint(length_min, length_max)
            angle = random.randint(angle_min, angle_max)
            stiffness = random.randint(stiffness_min, stiffness_max)
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

            self.add_linear_body(shape, density, (x, y), body, stiffness, damping,
                                 radius=radius, length=length, angle_offset=angle)

        # Delete body with joint
        elif rule == 3:
            if len(self.bodies) > 1:
                if kwargs.get('removal') is not None:
                    removal = kwargs['removal']
                else:
                    self.rule_check()
                    removal = self.deletable_bodies[random.randint(0, len(self.deletable_bodies)-1)]
                self.remove_body_with_joint(removal[0], removal[1], removal[2])

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

        # Change body dimensions
        elif rule == 5:
            if len(self.bodies) > 0:
                body = random.randint(0, len(self.bodies)-1)
                radius = random.randint(radius_min, radius_max)
                length = random.randint(length_min, length_max)
                angle = random.randint(angle_min, angle_max)
                self.change_body_dimensions(body, radius=radius, length=length, angle=angle)

        # Change body density
        elif rule == 6:
            body = random.randint(0, len(self.bodies)-1)
            density = random.randint(density_min, density_max)
            self.change_body_density(body, density)

        # Relocate body with joint
        elif rule == 7:
            if len(self.bodies) > 1:
                self.rule_check()
                removal = self.deletable_bodies[random.randint(0, len(self.deletable_bodies) - 1)]
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

        # Swap bodies
        elif rule == 8:
            if len(self.bodies) > 1:
                body_a = random.randint(0, len(self.bodies)-1)
                while True:
                    body_b = random.randint(0, len(self.bodies)-1)
                    if body_a != body_b:
                        break
                self.swap_bodies(body_a, body_b)

        # Add mooring system
        elif rule == 9:
            if len(self.bodies) > 0:
                x = random.randint(x_min, x_max)
                y = mooring_depth
                body = random.randint(0, len(self.bodies)-1)
                stiffness = random.randint(stiffness_min, stiffness_max)
                damping = random.randint(damping_min, damping_max)
                self.add_mooring_system((x, y), body, stiffness, damping)

        # Delete mooring system
        elif rule == 10:
            if len(self.mooring_attachment_points) > 0:
                mooring = random.randint(0, len(self.mooring_attachment_points)-1)
                self.remove_mooring_system(mooring)

        # Relocate cable attachment point
        elif rule == 11:
            if len(self.mooring_attachment_points) > 0:
                mooring = random.randint(0, len(self.mooring_attachment_points)-1)
                body = random.randint(0, len(self.bodies)-1)
                self.relocate_mooring_cable_attachment(mooring, body)

        # Relocate fixed body
        elif rule == 12:
            # print("Attachment points: ", self.mooring_attachment_points)
            # print("Fixed bodies: ", self.fixed_bodies)
            # print("Cable bodies: ", self.cable_bodies)
            if len(self.mooring_attachment_points) > 0:
                x = random.randint(x_min, x_max)
                y = mooring_depth
                mooring = random.randint(0, len(self.mooring_attachment_points)-1)
                self.relocate_mooring_fixed_body(mooring, (x, y))

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
        return validity

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
                    if distance == body_radius + test_body_radius + self.pto_size:
                        if pos_to_check == 0:
                            if test_body_pos[0] < body_pos[0] + distance:
                                valid_location = True
                            else:
                                valid_location = False
                                break
                        elif pos_to_check == 1:
                            if test_body_pos[1] < body_pos[1] + distance:
                                valid_location = True
                            else:
                                valid_location = False
                                break
                        elif pos_to_check == 2:
                            if test_body_pos[0] > body_pos[0] - distance:
                                valid_location = True
                            else:
                                valid_location = False
                                break
                        elif pos_to_check == 3:
                            if test_body_pos[1] > body_pos[1] - distance:
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
