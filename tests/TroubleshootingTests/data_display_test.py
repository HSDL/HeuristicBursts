import heuristic_bursts.agent
import heuristic_bursts.team
import heuristic_bursts.options

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import wec.wec
import wec.wec_visual
import time

import numpy

options = heuristic_bursts.options.Options()

# Instantiate team
team = heuristic_bursts.team.Team(options)

# Set team tier preferences
for agent in team.agent_list:
    agent.tier_weights = [0.0, 1.0]

# Run team for number of iterations listed in Options
team.run()

# Perform one last team interaction
team.interact()

# Take the solution of Agent 1 as the solution for the team
solution = team.agent_list[0].current_results
quality = team.agent_list[0].current_solution_quality
rules = team.agent_list[0].current_solution.applied_rules
all_qualities = team.agent_list[0].all_solution_qualities

print("Solution Results:", solution)
print("Solution Quality:", quality)
print("Rules Applied:", rules)

display = wec.wec_visual.wec_visual()

display.display(team.agent_list[0].current_solution)
time.sleep(1)
display.wait_to_continue()

plt.figure(1)
plt.subplot(111)
plt.xlabel('Iteration Number')
plt.ylabel('Solution Quality [W/kg]')
plt.title('Solution Quality Using Only Lower-Tier Rules (100 Iterations per Agent)')
plt.plot(all_qualities)

plt.show()

display.wait_to_continue()
agent.current_solution.display_visual = True
agent.current_solution.evaluate()
