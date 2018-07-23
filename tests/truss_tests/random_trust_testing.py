import truss.truss
import truss.truss_visual

import time

def test_simulation_of_truss():
    design = truss.truss.Truss()
    display = truss.truss_visual.truss_visual()
    display.display(design)

    time.sleep(1)
    display.wait_to_continue()

    design.add_joint([10, 12.5, 0])
    design.add_member(4, 5)
    design.add_member(1, 5)
    design.add_joint([5, 7.5, 0])
    design.add_member(4, 6)
    design.add_member(3, 6)
    design.add_member(5, 6)

    display.display(design)
    time.sleep(1)
    display.wait_to_continue()

    design.move_free_joint_rule(3, [0, -2.5, 0])
    design.move_free_joint_rule(4, [0, -2.5, 0])
    design.move_free_joint_rule(6, [0, -2.5, 0])

    display.display(design)
    time.sleep(1)
    display.wait_to_continue()

test_simulation_of_truss()
