from __future__ import division
from heuristic_bursts.abstract_base_solution import AbstractBaseSolution

import random
import copy
import numpy as np
import math

from matplotlib.tri import Triangulation


class Truss(AbstractBaseSolution):
    num_lowtier_rules = 7
    num_hightier_rules = 5

    # Yield strength of steel
    Fy = 344 * pow(10, 6)

    # Elastic modulus of steel
    E = 210 * pow(10, 9)

    # Outer diameters of the optional sizes, in meters
    OUTER_DIAM = [(x + 10.0) / 1000 for x in range(100)]

    # Thickness of the wall sections of optional sizes, in meters
    THICK = [d / 15 for d in OUTER_DIAM]

    # Cross sectional area in m^2
    AREA_SEC = [math.pi * pow(d / 2, 2) - math.pi * pow(d / 2 - d / 15, 2) for d in OUTER_DIAM]

    # Moment of inertia, in m^4
    I_SEC = [math.pi * (pow(d, 4) - pow((d - 2 * d / 15), 4)) / 64 for d in OUTER_DIAM]

    # Weight per length, kg/ft
    WEIGHT = [a * 7870 for a in AREA_SEC]

    def __init__(self):
        # Save number of joints
        self.n = 5

        self.em = np.zeros(100)
        self.ej = np.zeros(100)

        # Create first set of nodal coordinates
        self.coord = np.array([[0, 0, 0], [5, 0, 0], [2.5, 0, 0], [1.25, 3, 0], [3.75, 3, 0]])

        self.fixed_joints = [0, 1]
        self.force_joints = [2]
        self.undeletable_joints = []
        for joint in self.fixed_joints:
            self.undeletable_joints.append(joint)
        for joint in self.force_joints:
            self.undeletable_joints.append(joint)

        # Create connections for first set of nodes
        self.con = np.array([[0, 2], [1, 2], [0, 3], [1, 4], [2, 3], [2, 4], [3, 4]])

        self.dof = np.zeros(len(self.coord))

        # Store final number of members
        self.m = len(self.con)

        # Initialize truss sizes
        self.sizes = np.ones(len(self.con)) * 50

        # Establish the connectivity matrix
        self.con_mat = np.zeros([self.n, self.n])
        for member in self.con:
            self.con_mat[member[0], member[1]] = 1.0
            self.con_mat[member[1], member[0]] = 1.0

        # Target factor of safety
        self.target_fos = 1.0

        # Evaluate the truss
        self.force = [0.0]
        self.fos = np.array([0.0])
        self.mass = 0.0

        # List of applied rules
        self.applied_rules = []
        self.stable_system = True

    # Basic functions needed to implement rules below

    def add_joint(self, xy):
        # Increase number of joints
        self.n += 1

        temp = np.zeros([self.n, self.n])
        for i in range(self.n - 1):
            for j in range(self.n - 1):
                temp[i, j] = self.con_mat[i, j]

        self.con_mat = temp

        #         self.con_mat.resize(self.n, self.n)
        self.coord = np.vstack([self.coord, xy])
        b = [max(c) + 0.001 * min(c) for c in self.con]
        # if len(b) > 0:
        #     ratio = float(len(b)) / float(len(np.unique(b)))
        #     if ratio > 1.0:
        #         print("Add Joint:" + str(ratio))

    def add_member(self, a, b):
        self.con = np.vstack([self.con, np.array([a, b])])
        self.con_mat[a, b] = 1.0
        self.con_mat[b, a] = 1.0
        self.sizes = np.hstack([self.sizes, np.array(50.0)])
        self.m += 1
        d = [max(c) + 0.001 * min(c) for c in self.con]
        # if len(d) > 0:
        #     ratio = float(len(d)) / float(len(np.unique(d)))
        #     if ratio > 1.0:
        #         print("Add Member 2:" + str(ratio))
        #         print(len(self.con.T) / 2, sum(self.con_mat) / 2)

    def remove_joint(self, j):
        # Remove from coordinate list
        self.coord = np.delete(self.coord, j, 0)

        # Remove row and column from conn_mat
        self.con_mat = np.delete(self.con_mat, j, 0)
        self.con_mat = np.delete(self.con_mat, j, 1)

        # Remove connected members
        row, _ = np.where(self.con == j)
        self.con = np.delete(self.con, row, 0)
        self.sizes = np.delete(self.sizes, row)
        self.m -= len(row)

        # Decrement connections appropriately
        for i in range(self.m):
            if self.con[i, 0] > j:
                self.con[i, 0] -= 1
            if self.con[i, 1] > j:
                self.con[i, 1] -= 1

        # Decrement number of joints
        self.n -= 1
        b = [max(c) + 0.001 * min(c) for c in self.con]
        # if len(b) > 0:
        #     ratio = float(len(b)) / float(len(np.unique(b)))
        #     if ratio > 1.0:
        #         print("Del Joint:" + str(ratio))

    def remove_member(self, j):
        self.con_mat[self.con[j, 1], self.con[j, 0]] = 0.0
        self.con_mat[self.con[j, 0], self.con[j, 1]] = 0.0
        self.con = np.delete(self.con, j, 0)
        self.sizes = np.delete(self.sizes, j)
        self.m -= 1
        b = [max(c) + 0.001 * min(c) for c in self.con]
        # if len(b) > 0:
        #     ratio = float(len(b)) / float(len(np.unique(b)))
        #     if ratio > 1.0:
        #         print("Del Member:" + str(ratio))

    def move_joint(self, j, dxy):
        self.coord[j, :] += dxy
        b = [max(c) + 0.001 * min(c) for c in self.con]
        # if len(b) > 0:
        #     ratio = float(len(b)) / float(len(np.unique(b)))
        #     if ratio > 1.0:
        #         print("Move Joint:" + str(ratio))

    def change_member_size(self, j, dsize):
        self.sizes[j] += dsize
        if self.sizes[j] > 99.0:
            self.sizes[j] = 99.0
        elif self.sizes[j] < 0.0:
            self.sizes[j] = 0.0

    #### NOT SURE WHAT THESE ARE FOR YET
    def joint_violation(self, i):
        x = self.coord[0, i]
        y = self.coord[1, i]
        if x > -2:
            if x < -0.5:
                if y > -1.25:
                    if y < 2.5:
                        if y < -0.75:
                            return True
                        else:
                            if x > -1.0:
                                return True

        return False

    def member_violation(self, i, pg):
        pgs = Polygon(pg)
        x1 = self.coord[0, self.con[0, i]]
        y1 = self.coord[1, self.con[0, i]]
        x2 = self.coord[0, self.con[1, i]]
        y2 = self.coord[1, self.con[1, i]]

        xys = LineString([(x1, y1), (x2, y2)])
        inx = pgs.intersection(xys)

        return inx.length

    # Functions needed for evaluation below
    def evaluate(self):
        self.mass_eval()
        self.fos_eval()
        # self.quality_eval()

        return [self.mass, self.fos, self.target_fos]

    def mass_eval(self):
        """This function calculates the mass of the truss"""
        # Calculate lengths
        L = np.zeros(self.m)
        for i in range(self.m):
            L[i] = np.linalg.norm(self.coord[self.con[i, 0], :] - self.coord[self.con[i, 1], :])

        # Calculate total mass
        self.mass = 0
        for i in range(self.m):
            self.mass += L[i]*self.WEIGHT[int(self.sizes[i])]

    def fos_eval(self):
        support = np.array([[1, 1, 1], [1, 1, 1]]).T
        self._single_fos_eval(support)

    def _single_fos_eval(self, support):
        D = {}

        D["Re"] = support
        for _ in range(self.n - 2):
            D["Re"] = np.column_stack([D["Re"], [0, 0, 1]])

        # Add the appropriate loads
        D["Load"] = np.zeros([3, self.n])
        # D["Load"][1, 2] = -200000.0
        D["Load"][1, 2] = -250000.0

        # Add the area information from truss structure
        D["A"] = []
        for member_size in self.sizes:
            D["A"].append(self.AREA_SEC[int(member_size)])
        D["Coord"] = self.coord.T
        D["Con"] = self.con.T
        D["E"] = self.E * np.ones(self.m)

        # Do force analysis
        try:
            self.force, U, R = self._force_eval(D)
            self.stable = True
        except np.linalg.LinAlgError:
            self.force = np.ones(self.m) * pow(10, 16)
            self.stable = False

        # Calculate lengths
        L = np.zeros(self.m)
        for i in range(self.m):
            L[i] = np.linalg.norm(D["Coord"][:, D["Con"][0, i]] - D["Coord"][:, D["Con"][1, i]])

        # Calculate FOS's
        self.fos = np.zeros(self.m)
        for i in range(len(self.force)):
            self.fos[i] = D["A"][i] * self.Fy / self.force[i]
            if self.fos[i] < 0:
                self.fos[i] = min(math.pi * math.pi * self.E * self.I_SEC[int(self.sizes[i] - 1)] / (L[i] * L[i]) / -self.force[i],
                                  -self.fos[i])

        # Make sure loads and supports are connected
        for i in range(3):
            if np.size(np.where(self.con == i)) == 0:
                self.fos = np.zeros(self.m)

        if np.isnan(sum(self.fos)):
            self.fos = np.zeros(self.m)

        for i in range(self.m):
            if np.isinf(self.fos[i]):
                self.fos[i] = pow(10, 10)

    def _force_eval(self, D):
        Tj = np.zeros([3, np.size(D["Con"], axis=1)])
        w = np.array([np.size(D["Re"], axis=0), np.size(D["Re"], axis=1)])
        SS = np.zeros([3*w[1], 3*w[1]])
        U = 1.0 - D["Re"]

        # This identifies joints that are unsupported, and can therefore be loaded
        ff = np.where(U.T.flat == 1)[0]

        # Step through the each member in the truss, and build the global stiffness matrix
        for i in range(np.size(D["Con"], axis=1)):
            H = D["Con"][:, i]
            C = D["Coord"][:, H[1]] - D["Coord"][:, H[0]]
            Le = np.linalg.norm(C)
            T = C/Le
            s = np.outer(T, T)
            G = D["E"][i]*D["A"][i]/Le
            ss = G*np.concatenate((np.concatenate((s, -s), axis=1), np.concatenate((-s, s), axis=1)), axis=0)
            Tj[:, i] = G*T
            e = list(range(int(3*H[0]), int(3*H[0] + 3))) + list(range(int(3*H[1]), int(3*H[1] + 3)))
            for ii in range(6):
                for j in range(6):
                    SS[e[ii], e[j]] += ss[ii, j]

        SSff = np.zeros([len(ff), len(ff)])
        for i in range(len(ff)):
            for j in range(len(ff)):
                SSff[i, j] = SS[ff[i], ff[j]]

        Loadff = D["Load"].T.flat[ff]

        Uff = np.linalg.solve(SSff, Loadff)

        ff = np.where(U.T==1)
        for i in range(len(ff[0])):
            U[ff[1][i], ff[0][i]] = Uff[i]
        F = np.sum(np.multiply(Tj, U[:, D["Con"][1, :].astype(int)] - U[:, D["Con"][0, :].astype(int)]), axis=0)
        if np.linalg.cond(SSff) > pow(10, 10):
            F *= pow(10, 10)
        R = np.sum(SS*U.T.flat[:], axis=1).reshape([w[1], w[0]]).T

        return F, U, R

    # Lower-tier rules below

    # LT rule 1
    def split_free_member_rule(self, member_index):
        # Keep track of connected joints
        member = self.con[member_index]
        member_size = self.sizes[member_index]
        joint_a = member[0]
        joint_b = member[1]

        # Delete member
        self.remove_member(member_index)

        # Add new joint between old ones
        joint_a_coord = self.coord[joint_a]
        joint_b_coord = self.coord[joint_b]
        new_joint_coord = [(joint_a_coord[0]+joint_b_coord[0])/2, (joint_a_coord[1]+joint_b_coord[1])/2, (joint_a_coord[2]+joint_b_coord[2])/2]
        self.add_joint(new_joint_coord)
        new_joint = len(self.coord) - 1

        # Add new members connecting new joint to previous connection joints
        self.add_member(joint_a, new_joint)
        temp_size = self.sizes[len(self.sizes)-1]
        dsize = member_size - temp_size
        self.change_member_size(len(self.sizes)-1, dsize)
        self.add_member(joint_b, new_joint)
        self.change_member_size(len(self.sizes)-1, dsize)

        # Add member between new joint and any joint that previous connection joints share
        for i in range(0, len(self.con_mat)):
            if self.con_mat[i, joint_a] == 1.0 and self.con_mat[i, joint_b] == 1.0 and i != new_joint:
                self.add_member(i, new_joint)

    # LT rule 2
    def join_free_members_rule(self, set_to_join):
        print('join:', set_to_join)
        self.remove_joint(set_to_join[0])
        if set_to_join[1] > set_to_join[0]:
            set_to_join[1] -= 1
        if set_to_join[2] > set_to_join[0]:
            set_to_join[2] -= 1
        self.add_member(set_to_join[1], set_to_join[2])

    # LT rule 3
    def add_free_joint_rule(self, coordinates, connection_list):
        self.add_joint(coordinates)
        new_joint = len(self.coord) - 1
        for joint in connection_list:
            if joint != new_joint:
                self.add_member(joint, new_joint)

    # LT rule 4
    def remove_free_joint_rule(self, joint_index):
        self.remove_joint(joint_index)

    # LT rule 5
    def switch_diagonal_member_joints_rule(self, member_index, joint_a, joint_b):
        self.remove_member(member_index)
        self.add_member(joint_a, joint_b)

    # LT rule 6
    def move_free_joint_rule(self, joint_index, d_coordinates):
        self.move_joint(joint_index, d_coordinates)

    # LT rule 7
    def change_member_size_rule(self, member_index, size_change):
        self.change_member_size(member_index, size_change)

    # Higher-tier rules below

    # HT rule 1
    def increase_complexity_rule(self, complexity_increase):
        for i in range(complexity_increase):
            self.rule_check()
            lt_rule_index = random.randint(0, 1)
            if lt_rule_index == 0:
                lt_rule = 1
            elif lt_rule_index == 1:
                lt_rule = 3
            self.lowtier_rule_perform(lt_rule)
            del self.applied_rules[-1]

    # HT rule 2
    def decrease_complexity_rule(self, complexity_decrease):
        for i in range(complexity_decrease):
            self.rule_check()
            if len(self.joinable_sets) > 0 and len(self.deletable_joints) > 0:
                lt_rule_index = random.randint(0, 1)
                if lt_rule_index == 0:
                    lt_rule = 2
                elif lt_rule_index == 1:
                    lt_rule = 4
                self.lowtier_rule_perform(lt_rule)
                del self.applied_rules[-1]
            elif len(self.joinable_sets) == 0 and len(self.deletable_joints) > 0:
                lt_rule = 4
                self.lowtier_rule_perform(lt_rule)
                del self.applied_rules[-1]
            elif len(self.joinable_sets) > 0 and len(self.deletable_joints) == 0:
                lt_rule = 2
                self.lowtier_rule_perform(lt_rule)
                del self.applied_rules[-1]
            else:
                break

    # HT rule 3
    def change_design_scale_rule(self, scale_multiplier, scale_type):
        if scale_type == 'member size':
            for j in range(len(self.sizes)):
                current_size = self.sizes[j]
                new_size = scale_multiplier*current_size
                if new_size < 1.0:
                    new_size = 1.0
                elif new_size > 99.0:
                    new_size = 99.0
                d_size = new_size - current_size
                self.change_member_size(j, d_size)
        elif scale_type == 'joint position':
            for j in range(len(self.coord)):
                current_coord = self.coord[j]
                new_coord = [current_coord[0], scale_multiplier*current_coord[1], current_coord[2]]
                d_coord = new_coord-current_coord
                self.move_joint(j, d_coord)

    # HT rule 4
    def replicate_pattern_rule(self, pattern):
        new_joint_coord = pattern[0]
        connecting_joint_a = pattern[1]
        connecting_joint_b = pattern[2]
        self.add_joint(new_joint_coord)
        new_joint_index = len(self.coord)-1
        self.add_member(connecting_joint_a, new_joint_index)
        self.add_member(connecting_joint_b, new_joint_index)

    # HT rule 5
    def standardize_rule(self, standardization_rate, base_member):
        base_member_size = self.sizes[base_member]
        for member_index in range(len(self.sizes)):
            member_size = self.sizes[member_index]
            size_change = standardization_rate*(base_member_size - member_size)
            self.change_member_size_rule(member_index, size_change)

    # Lower-tier random rule selection
    def lowtier_rule_select(self):
        rule = random.randint(1, self.num_lowtier_rules)
        return rule

    # Higher-tier random rule selection
    def hightier_rule_select(self):
        rule = random.randint(1, self.num_hightier_rules)
        return rule

    # Lower-tier rule perform
    def lowtier_rule_perform(self, rule, **kwargs):
        self.rule_check()

        max_new_dist = 3
        min_size = 0.0
        max_size = 99.0
        min_coord_shift = 0.01
        max_coord_shift = 1.0

        valid_rule = False

        # Split free member
        if rule == 1:
            if kwargs.get('member') is not None:
                member = kwargs['member']
            else:
                member = random.randint(0, len(self.con)-1)

            self.split_free_member_rule(member)
            self.applied_rules.append('L1')

        # Join free members
        elif rule == 2:
            if len(self.joinable_sets) > 0:
                if kwargs.get('set_to_join') is not None:
                    set_to_join = kwargs['set_to_join']
                else:
                    set_to_join = self.joinable_sets[random.randint(0, len(self.joinable_sets)-1)]
                self.join_free_members_rule(set_to_join)
                self.applied_rules.append('L2')

        # Add free joint
        elif rule == 3:
            if kwargs.get('coord') is not None:
                coord = kwargs['coord']
            else:
                min_x = 10
                max_x = -10
                min_y = 10
                max_y = -10
                for joint in self.coord:
                    if joint[0] < min_x:
                        min_x = joint[0]
                    if joint[0] > max_x:
                        max_x = joint[0]
                    if joint[1] < min_y:
                        min_y = joint[0]
                    if joint[1] > max_y:
                        max_y = joint[0]

                coord = [random.uniform(min_x - max_new_dist, max_x + max_new_dist), random.uniform(min_y - max_new_dist, max_y + max_new_dist), 0]

            connection_list = []

            for i in range(len(self.coord)):
                test_node = self.coord[i]
                intersection = False
                for j in range(len(self.con)):
                    a = self.coord[self.con[j][0]]
                    b = self.coord[self.con[j][1]]

                    if self.check_for_line_intersection(coord, test_node, a, b):
                        intersection = True
                if not intersection:
                    connection_list.append(i)

            self.add_free_joint_rule(coord, connection_list)
            self.applied_rules.append('L3')

        # Remove free joint
        elif rule == 4:
            if len(self.deletable_joints) > 0:
                if kwargs.get('joint') is not None:
                    joint = kwargs['joint']
                else:
                    joint = self.deletable_joints[random.randint(0, len(self.deletable_joints)-1)]

                self.remove_free_joint_rule(joint)
                self.applied_rules.append('L4')

        # Switch connection joints of diagonal members
        elif rule == 5:
            if len(self.re_diagonable_members) > 0:
                member = self.re_diagonable_members[random.randint(0, len(self.re_diagonable_members)-1)]
                member_index = member[0]
                new_joint_a = member[2][0]
                new_joint_b = member[2][1]
                self.switch_diagonal_member_joints_rule(member_index, new_joint_a, new_joint_b)
                self.applied_rules.append('L5')

        # Move free joint
        elif rule == 6:
            if kwargs.get('joint') is not None:
                joint = kwargs['joint']
            else:
                joint_index = random.randint(0, len(self.moveable_joints)-1)
                joint = self.moveable_joints[joint_index]
                print('joint', joint)
            if kwargs.get('coord') is not None:
                new_coord = kwargs['coord']
                old_coord = self.coord[joint]
                d_coord = new_coord - old_coord
            elif kwargs.get('d_coord') is not None:
                d_coord = kwargs['d_coord']
            else:
                d_x = pow(-1,random.randint(1, 2))*random.uniform(min_coord_shift, max_coord_shift)
                d_y = pow(-1,random.randint(1, 2))*random.uniform(min_coord_shift, max_coord_shift)
                d_coord = [d_x, d_y, 0]
            self.move_free_joint_rule(joint, d_coord)
            attempts = 1
            if not self.is_valid():
                while not self.is_valid():
                    d_x = pow(-1, random.randint(1, 2)) * random.uniform(min_coord_shift, max_coord_shift) - d_coord[0]
                    d_y = pow(-1, random.randint(1, 2)) * random.uniform(min_coord_shift, max_coord_shift) - d_coord[1]
                    d_coord = [d_x, d_y, 0]
                    self.move_free_joint_rule(joint, d_coord)
                    attempts += 1
                    if attempts > 100:
                        break
            self.applied_rules.append('L6')

        # Resize member
        elif rule == 7:
            if kwargs.get('member') is not None:
                member = kwargs['member']
            else:
                member = random.randint(0, len(self.con)-1)

            if kwargs.get('size') is not None:
                new_size = kwargs['size']
            else:
                new_size = random.randint(min_size, max_size)

            old_size = self.sizes[member]
            d_size = new_size - old_size
            self.change_member_size_rule(member, d_size)
            self.applied_rules.append('L7')

    # Higher-tier rule perform
    def hightier_rule_perform(self, rule, **kwargs):
        self.rule_check()

        min_complexity_change = 1
        max_complexity_change = 3
        min_scale_multiplier = 0.75
        max_scale_multiplier = 1.25
        min_standardization_rate = 0.20
        max_standardization_rate = 1.00

        valid_rule = False

        # Increase complexity
        if rule == 1:
            if kwargs.get('complexity_change') is not None:
                complexity_change = kwargs['complexity_change']
            else:
                complexity_change = random.randint(min_complexity_change, max_complexity_change)
            self.increase_complexity_rule(complexity_change)
            self.applied_rules.append('H1')

        # Decrease complexity
        elif rule == 2:
            if kwargs.get('complexity_change') is not None:
                complexity_change = kwargs['complexity_change']
            else:
                complexity_change = random.randint(min_complexity_change, max_complexity_change)
            self.decrease_complexity_rule(complexity_change)
            self.applied_rules.append('H2')

        # Change design scale
        elif rule == 3:
            if kwargs.get('scale_multiplier') is not None:
                scale_multiplier = kwargs['scale_multiplier']
            else:
                scale_multiplier = random.uniform(min_scale_multiplier, max_scale_multiplier)
            if kwargs.get('scale_type') is not None:
                scale_type = kwargs['scale_type']
            else:
                scale_type_index = random.randint(0, 1)
                if scale_type_index == 0:
                    scale_type = 'member size'
                elif scale_type_index == 1:
                    scale_type = 'joint position'
            self.change_design_scale_rule(scale_multiplier, scale_type)
            self.applied_rules.append('H3')

        # Replicate pattern
        elif rule == 4:
            if len(self.patterns) > 0:
                pattern = self.patterns[random.randint(0, len(self.patterns)-1)]
                self.replicate_pattern_rule(pattern)
                self.applied_rules.append('H4')

        # Standardize
        elif rule == 5:
            if kwargs.get('standardization_rate') is not None:
                standardization_rate = kwargs['standardization_rate']
            else:
                standardization_rate = random.uniform(min_standardization_rate, max_standardization_rate)
            if kwargs.get('base_member') is not None:
                base_member = kwargs['base_member']
            else:
                base_member = random.randint(0, len(self.con)-1)
            self.standardize_rule(standardization_rate, base_member)
            self.applied_rules.append('H5')

    # Function used to identify where rules can be applied
    def rule_check(self):
        #TODO: CHECK THIS
        # Check which joints can be deleted
        self.deletable_joints = []
        for joint_index in range(0, len(self.con_mat)):
            deletable = True
            for check_index in range(0, len(self.con_mat)):
                if self.con_mat[joint_index, check_index] == 1.0:
                    if sum(self.con_mat[check_index]) < 3.0:
                        deletable = False
                        break
            if deletable and joint_index not in self.undeletable_joints:
                self.deletable_joints.append(joint_index)

        # Check which joints can be deleted to join members
        self.joinable_sets = []
        for joint_index in range(0, len(self.coord)):
            connected_joints = self.con_mat[joint_index]
            for i in range(len(connected_joints)):
                connection = connected_joints[i]
                if connection == 1:
                    i_mat = self.con_mat[i]
                    for j in range(len(i_mat)):
                        if j != joint_index and j != i:
                            match = 0
                            j_mat = self.con_mat[j]
                            if j_mat[joint_index] == 1:
                                for k in range(len(i_mat)):
                                    if i_mat[k] == j_mat[k] and i_mat[k] == 1:
                                        match += 1
                                if match >= 2 and i_mat[j] != 1:
                                    set = [joint_index, i, j]
                                    already_in_set = False
                                    for check in self.joinable_sets:
                                        if check[0] == set[0]:
                                            if (check[1] == set[1] and check[2] == set[2]) or (check[1] == set[2] and check[2] == set[1]):
                                                already_in_set = True
                                    if not already_in_set and set[0] not in self.undeletable_joints:
                                        self.joinable_sets.append(set)

        # Check to see if performing join function would leave nodes with only 1 member
        removable_list_sets = []
        for set_index in range(len(self.joinable_sets)):
            set = self.joinable_sets[set_index]
            for i in range(len(self.con_mat)):
                leftover = sum(self.con_mat[i]) - self.con_mat[i][set[0]]
                if i == set[1] or i == set[2]:
                    leftover += 1
                if leftover < 2:
                    removable_list_sets.append(set_index)
                    break
        while len(removable_list_sets) > 0:
            del self.joinable_sets[removable_list_sets[0]]
            del removable_list_sets[0]
            for i in range(len(removable_list_sets)):
                removable_list_sets[i] -= 1

        # Check joinable sets to see if there is collision with new members
        removable_list_sets = []
        for set_index in range(len(self.joinable_sets)):
            set = self.joinable_sets[set_index]
            node_a = self.coord[set[1]]
            node_b = self.coord[set[2]]

            for con in self.con:
                node_c = self.coord[con[0]]
                node_d = self.coord[con[1]]
                #
                # print('a', set[1])
                # print('b', set[2])
                # print('c', con[0])
                # print('d', con[1])

                if not ((set[1] == con[0] and set[2] == con[1]) or (set[1] == con[1] and set[2] == con[0])):
                    intersection = self.check_for_line_intersection(node_a, node_b, node_c, node_d)
                    # TODO: CHECK THIS!
                    if set[0] == con[0] or set[0] == con[1]:
                        intersection = False
                    if intersection:
                        removable_list_sets.append(set_index)
                        break
        while len(removable_list_sets) > 0:
            del self.joinable_sets[removable_list_sets[0]]
            del removable_list_sets[0]
            for i in range(len(removable_list_sets)):
                removable_list_sets[i] -= 1

        # Check which diagonal members can be repositioned
        self.re_diagonable_members = []
        for member_index in range(0, len(self.con)):
            member = self.con[member_index]
            check_a = self.con_mat[member[0]]
            check_b = self.con_mat[member[1]]
            match = []
            for i in range(0, len(check_a)):
                if check_a[i] == check_b[i] and check_a[i] == 1:
                    match.append(i)
            if len(match) >= 2:
                pairs = []
                for j in range(0, len(match)):
                    a = match[j]
                    for k in range(0, len(match)):
                        b = match[k]
                        if a != b and a < b:
                            pairs.append((a, b))
                for pair in pairs:
                    self.re_diagonable_members.append([member_index, member, pair])

        # Check to see if any potential new member already exists
        removable_list_members = []
        for test_member_index in range(len(self.re_diagonable_members)):
            test_member = self.re_diagonable_members[test_member_index][2]
            for check_member_index in range(len(self.con)):
                check_member = self.con[check_member_index]
                if (test_member[0] == check_member[0] and test_member[1] == check_member[1]) or (test_member[0] == check_member[1] and test_member[1] == check_member[0]):
                    removable_list_members.append(test_member_index)
        while len(removable_list_members) > 0:
            del self.re_diagonable_members[removable_list_members[0]]
            del removable_list_members[0]
            for i in range(len(removable_list_members)):
                removable_list_members[i] -= 1

        # Check re-diagonable members to see if there is collision with existing members
        removable_list_members = []
        for member_index in range(len(self.re_diagonable_members)):
            member = self.re_diagonable_members[member_index]
            node_a_index = member[2][0]
            node_b_index = member[2][1]
            node_a = self.coord[node_a_index]
            node_b = self.coord[node_b_index]

            mem_to_delete = member[1]

            for con in self.con:
                node_c_index = con[0]
                node_d_index = con[1]
                node_c = self.coord[node_c_index]
                node_d = self.coord[node_d_index]

                intersection = self.check_for_line_intersection(node_a, node_b, node_c, node_d)
                if (node_c_index == member[1][0] and node_d_index == member[1][1]) or (node_c_index == member[1][1] and node_d_index == member[1][0]):
                    intersection = False
                if intersection:
                    removable_list_members.append(member_index)
                    break
        while len(removable_list_members) > 0:
            del self.re_diagonable_members[removable_list_members[0]]
            del removable_list_members[0]
            for i in range(len(removable_list_members)):
                removable_list_members[i] -= 1

        # Check which joints can be moved
        self.moveable_joints = []
        for i in range(len(self.coord)):
            if i not in self.undeletable_joints:
                self.moveable_joints.append(i)

        # Check which patterns are valid
        self.patterns = []
        potential_patterns = []
        for i in range(len(self.con)):
            member = self.con[i]
            joint_a = member[0]
            joint_b = member[1]
            for j in range(len(self.con_mat)):
                if self.con_mat[joint_a][j] == 1.0 and self.con_mat[joint_b][j] == 1.0:
                    potential_patterns.append([joint_a, joint_b, j])
        # Identify locations of patterns
        for i in range(len(potential_patterns)):
            pattern = potential_patterns[i]
            coord_a = self.coord[pattern[0]]
            coord_b = self.coord[pattern[1]]
            coord_c = self.coord[pattern[2]]

            # Figure out coordinates of reflected point c
            if coord_b[0] - coord_a[0] != 0 and coord_b[1] - coord_a[1] != 0:
                m1 = (coord_b[1]-coord_a[1])/(coord_b[0]-coord_a[0])
                b1 = coord_a[1] - m1*coord_a[0]
                m2 = -1/m1
                b2 = coord_c[1] - m2*coord_c[0]
                x_int = (b2 - b1)/(m1-m2)
                dx = coord_c[0] - x_int
                x_new = x_int - dx
                y_new = m2*x_new + b2
            elif coord_b[1] - coord_a[1] == 0:
                x_new = coord_c[0]
                dy = coord_c[1] - coord_a[1]
                y_new = coord_a[1] - dy
            else:
                dx = coord_c[0] - coord_a[0]
                x_new = coord_a[0] - dx
                y_new = coord_c[1]

            coord_d = [x_new, y_new, 0]
            self.patterns.append([coord_d, pattern[0], pattern[1]])

        # Check patterns to see if there is any collisions with existing members
        removable_list_patterns = []
        for pattern_index in range(len(self.patterns)):
            pattern = self.patterns[pattern_index]
            new_node_coord = pattern[0]
            node_a = self.coord[pattern[1]]
            node_b = self.coord[pattern[2]]

            for member in self.con:
                node_c = self.coord[member[0]]
                node_d = self.coord[member[1]]

                intersection = self.check_for_line_intersection(new_node_coord, node_a, node_c, node_d)

                # print('')
                # print('new', new_node_coord)
                # print('a', node_a)
                # print('b', node_b)
                # print('c', node_c)
                # print('d', node_d)
                # print('new/a', intersection)

                if intersection:
                    removable_list_patterns.append(pattern_index)
                    break
                else:
                    intersection = self.check_for_line_intersection(new_node_coord, node_b, node_c, node_d)
                    # print('new/b', intersection)
                    if intersection:
                        removable_list_patterns.append(pattern_index)
                        break
        while len(removable_list_patterns) > 0:
            del self.patterns[removable_list_patterns[0]]
            del removable_list_patterns[0]
            for i in range(len(removable_list_patterns)):
                removable_list_patterns[i] -= 1

    # Checks entire design to ensure design is valid
    def is_valid(self):
        validity = True

        for i in range(len(self.con)):
            for j in range(len(self.con)):
                if i != j:
                    member_1 = self.con[i]
                    member_2 = self.con[j]
                    a_idx = member_1[0]
                    b_idx = member_1[1]
                    c_idx = member_2[0]
                    d_idx = member_2[1]
                    node_a = self.coord[a_idx]
                    node_b = self.coord[b_idx]
                    node_c = self.coord[c_idx]
                    node_d = self.coord[d_idx]
                    intersection = self.check_for_line_intersection(node_a, node_b, node_c, node_d)
                    if a_idx == c_idx or a_idx == d_idx or b_idx == c_idx or b_idx == d_idx:
                        intersection = False
                    if intersection:
                        validity = False
                        break
            if not validity:
                break

        return validity

    # Check to see if connections between node pairs intersect
    def check_for_line_intersection(self, node_a, node_b, node_c, node_d):
        do_intersect = False

        # If slopes and y-intercepts match, check to see if collinear lines are overlapping
        if abs(node_b[0]-node_a[0]) > 0:
            m1 = (node_b[1]-node_a[1])/(node_b[0]-node_a[0])
            b1 = node_a[1] - m1 * node_a[0]
        else:
            m1 = 'undefined'
            b1 = 'undefined'
        if abs(node_d[0] - node_c[0]) > 0:
            m2 = (node_d[1]-node_c[1])/(node_d[0]-node_c[0])
            b2 = node_c[1] - m2 * node_c[0]
        else:
            m2 = 'undefined'
            b2 = 'undefined'
        if m1 == m2 and b1 == b2:
            if self.is_between_values(node_a[0], node_c[0], node_d[0]) or self.is_between_values(node_b[0], node_c[0], node_d[0])\
                    or self.is_between_values(node_c[0], node_a[0], node_b[0]) or self.is_between_values(node_d[0], node_a[0], node_b[0]):
                    do_intersect = True
            if (node_a[0] == node_c[0] and self.is_between_values(node_b[0], node_c[0], node_d[0])) or \
                    (node_a[0] == node_c[0] and self.is_between_values(node_d[0], node_a[0], node_b[0])) or \
                    (node_a[0] == node_d[0] and self.is_between_values(node_b[0], node_c[0], node_d[0])) or \
                    (node_a[0] == node_d[0] and self.is_between_values(node_c[0], node_a[0], node_b[0])) or \
                    (node_b[0] == node_c[0] and self.is_between_values(node_a[0], node_c[0], node_d[0])) or \
                    (node_b[0] == node_c[0] and self.is_between_values(node_d[0], node_a[0], node_b[0])) or \
                    (node_b[0] == node_d[0] and self.is_between_values(node_a[0], node_c[0], node_d[0])) or \
                    (node_b[0] == node_d[0] and self.is_between_values(node_c[0], node_a[0], node_b[0])):
                    do_intersect = True
        else:
            line_1 = self.line_creation(node_a, node_b)
            line_2 = self.line_creation(node_c, node_d)
            intersect = self.intersection(line_1, line_2)
            rounding_error = 0.000005
            if intersect is not False:
                if (self.is_between_values(intersect[0], node_a[0], node_b[0]) or intersect[0] == node_a[0])\
                        and (self.is_between_values(intersect[1], node_a[1], node_b[1]) or intersect[1] == node_a[1])\
                        and (self.is_between_values(intersect[0], node_c[0], node_d[0]) or intersect[0] == node_c[0])\
                        and (self.is_between_values(intersect[1], node_c[1], node_d[1]) or intersect[1] == node_c[1]):
                    if not ((self.is_between_values(intersect[0], node_a[0] - rounding_error, node_a[0] + rounding_error) and
                             self.is_between_values(intersect[1], node_a[1] - rounding_error, node_a[1] + rounding_error)) or
                            (self.is_between_values(intersect[0], node_b[0] - rounding_error, node_b[0] + rounding_error) and
                             self.is_between_values(intersect[1], node_b[1] - rounding_error, node_b[1] + rounding_error)) or
                            (self.is_between_values(intersect[0], node_c[0] - rounding_error, node_c[0] + rounding_error) and
                             self.is_between_values(intersect[1], node_c[1] - rounding_error, node_c[1] + rounding_error)) or
                            (self.is_between_values(intersect[0], node_d[0] - rounding_error, node_d[0] + rounding_error) and
                             self.is_between_values(intersect[1], node_d[1] - rounding_error, node_d[1] + rounding_error))):
                        do_intersect = True
        return do_intersect

    # Create imaginary lines to test for intersection
    def line_creation(self, a, b):
        A = (a[1] - b[1])
        B = (b[0] - a[0])
        C = (a[0]*b[1] - b[0]*a[1])
        return A, B, -C

    # Test line segments for intersection point
    def intersection(self, a, b):
        D = a[0]*b[1] - a[1]*b[0]
        Dx = a[2]*b[1] - a[1]*b[2]
        Dy = a[0]*b[2] - a[2]*b[0]

        if D != 0:
            x = Dx/D
            y = Dy/D
            return x, y
        else:
            return False

    # See if value is between two other values
    def is_between_values(self, num, a, b):
        minimum = min(a, b)
        maximum = max(a, b)
        if minimum < num < maximum:
            is_between = True
        else:
            is_between = False
        return is_between

    # Function needed to make copy of Truss object by agent
    def __deepcopy__(self, memo):
        deepcopy_method = self.__deepcopy__
        self.__deepcopy__ = None
        cp = copy.deepcopy(self, memo)
        self.__deepcopy__ = deepcopy_method
        return cp
