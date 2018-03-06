import heuristic_bursts.agent
import heuristic_bursts.team
import heuristic_bursts.options

# file = open("test_simulation_file.txt", "a")
# file.write("{Solution Metrics, Solution Quality, Number of Rules Applied, Rules Applied}\n")
# file.close()

options = heuristic_bursts.options.Options()

# Instantiate team
team = heuristic_bursts.team.Team(options)

# Set team tier preferences
for agent in team.agent_list:
    agent.tier_weights = [1.0, 0.0]

# Run team for number of iterations listed in Options
team.run()

# Perform one last team interaction
team.interact()

# Take the solution of Agent 1 as the solution for the team
solution = team.agent_list[0].current_results
quality = team.agent_list[0].current_solution_quality
num_rules = len(team.agent_list[0].current_solution.applied_rules)
rules = team.agent_list[0].current_solution.applied_rules

# Save results to file
file = open("test_simulation_file.txt", "a")
file.write(str(solution)+', '+str(quality)+', '+str(num_rules)+', '+str(rules)+'\n')
file.close()
