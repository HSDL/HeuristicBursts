import heuristic_bursts.agent
import heuristic_bursts.team
import heuristic_bursts.options

import truss.truss
import truss.truss_visual
import time

import csv

options = heuristic_bursts.options.Options()

# with open('both_tiers_500_iterations.csv', 'w') as sim_data_file:
#     fieldnames = ['repetition', 'iteration', 'rule tier', 'rule number', 'quality before rule', 'quality after rule',
#                   'current solution quality', 'rule acceptance', 'lower tier preference', 'higher tier preference',
#                   'error', 'probability array', 'current solution mass', 'current solution min fos', 'current solution target fos']
#
#     csv_writer = csv.DictWriter(sim_data_file, fieldnames=fieldnames)
#     csv_writer.writeheader()

for sim_num in range(0, 52):

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
    solution = team.agent_list[0].current_solution
    results = team.agent_list[0].current_results
    quality = team.agent_list[0].current_solution_quality
    rules = team.agent_list[0].current_solution.applied_rules
    all_qualities = team.agent_list[0].all_solution_qualities
    simulation_data = team.agent_list[0].simulation_data

    with open('both_tiers_500_iterations.csv', 'r') as sim_data_file:
        csv_reader = csv.DictReader(sim_data_file)

        last_rep = -1

        for row in csv_reader:
            last_rep = int(row['repetition'])

    with open('both_tiers_500_iterations.csv', 'a') as sim_data_file:
        csv_writer = csv.writer(sim_data_file)

        for iteration_data in simulation_data:
            iteration_data[0] = last_rep + 1
            csv_writer.writerow(iteration_data)

    # team.display.wait_to_continue()
