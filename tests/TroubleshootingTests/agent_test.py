import heuristic_bursts.agent
import heuristic_bursts.options

import wec.wec
import wec.wec_visual
import time

options = heuristic_bursts.options.Options()
agent = heuristic_bursts.agent.Agent(options)
display = wec.wec_visual.wec_visual()

display.display(agent.current_solution)
# time.sleep(3)
# display.wait_to_continue()

for i in range(0, 20):
    print(agent.iteration_count)
    agent.iterate()
    display.display(agent.current_solution)
    # time.sleep(3)
    # display.wait_to_continue()

print('Number of Bodies in Final Solution:', len(agent.current_solution.bodies))
print('')

print('Final Solution Metrics:', agent.current_results)
print('Final Solution Quality:', agent.current_solution_quality)
print('Final Solution Applied Rules:', agent.current_solution.applied_rules)

display.wait_to_continue()
agent.current_solution.display_visual = True
agent.current_solution.evaluate()
