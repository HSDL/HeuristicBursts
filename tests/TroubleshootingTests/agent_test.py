import heuristic_bursts.agent
import heuristic_bursts.options

import wec.wec
import wec.wec_visual
import time

import numpy

options = heuristic_bursts.options.Options()
agent = heuristic_bursts.agent.Agent(options)
display = wec.wec_visual.wec_visual()

agent.tier_weights = [1.0, 0.0]

display.display(agent.current_solution)
# time.sleep(3)
# display.wait_to_continue()

for i in range(0, 75):
    print(agent.iteration_count)
    agent.iterate()
    display.display(agent.current_solution)
    print(agent.current_solution_quality)
    for body in agent.current_solution.bodies:
        print(body)
    # time.sleep(3)
    # display.wait_to_continue()

print('Final Solution Metrics:', agent.current_results)
print('Final Solution Quality:', agent.current_solution_quality)
print('Final Solution Applied Rules:', agent.current_solution.applied_rules)
print('')

for row in agent.simulation_data:
    print(row)

print('')

display.wait_to_continue()
agent.current_solution.display_visual = True
agent.current_solution.evaluate()
