import heuristic_bursts.agent
import heuristic_bursts.team
import heuristic_bursts.options

options = heuristic_bursts.options.Options()

for sim_num in range(0, 20):
    # Instantiate team
    team = heuristic_bursts.team.Team(options)

    # Set team tier preferences
    for agent in team.agent_list:
        agent.tier_weights = [0.5, 0.5]

    # Run team for number of iterations listed in Options
    team.run()

    # Perform one last team interaction
    team.interact()

    # Take the solution of Agent 1 as the solution for the team
    solution = team.agent_list[0].current_results
    quality = team.agent_list[0].current_solution_quality
    num_rules = len(team.agent_list[0].current_solution.applied_rules)
    rules = team.agent_list[0].current_solution.applied_rules
    all_qualities = team.agent_list[0].all_solution_qualities

    # Save results to file
    file = open("simulation_50low_50high.txt", "a")
    file.write(str(solution[0]) + ', ' + str(solution[1]) + ', ' + str(solution[2]) + ', ,,,' + str(quality) + ', ,,,' + str(num_rules) + ', ,,,'+str(rules) + '\n')
    file.close()

    file = open("simulation_50low_50high_qualities.txt", "a")
    file.write(str(all_qualities) + '\n')
    file.close()
