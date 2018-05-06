import heuristic_bursts.agent
import heuristic_bursts.team
import heuristic_bursts.options

import wec.wec
import wec.wec_visual
import time

import csv

options = heuristic_bursts.options.Options()

# with open('powerbuoy_optimization_500_iter.csv', 'w') as sim_data_file:
#     fieldnames = ['repetition', 'iteration', 'rule tier', 'rule number', 'quality before rule', 'quality after rule',
#                   'current solution quality', 'rule acceptance', 'lower tier preference', 'higher tier preference',
#                   'error']
#
#     csv_writer = csv.DictWriter(sim_data_file, fieldnames=fieldnames)
#     csv_writer.writeheader()

for sim_num in range(0, 50):

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
    solution = team.agent_list[0].current_solution
    results = team.agent_list[0].current_results
    quality = team.agent_list[0].current_solution_quality
    rules = team.agent_list[0].current_solution.applied_rules
    all_qualities = team.agent_list[0].all_solution_qualities
    simulation_data = team.agent_list[0].simulation_data

    # body_radii = []
    # rot_pto_damping = []
    # rot_pto_stiffness = []
    # lin_pto_damping = []
    # lin_pto_stiffness = []
    # for body in solution.bodies:
    #     body_radii.append(body['radius'])
    # for pto in solution.rotary_ptos_data:
    #     rot_pto_damping.append(pto['damping'])
    #     rot_pto_stiffness.append(pto['stiffness'])
    # for pto in solution.linear_ptos_data:
    #     lin_pto_damping.append(pto['damping'])
    #     lin_pto_stiffness.append(pto['stiffness'])

    with open('powerbuoy_optimization_500_iter.csv', 'r') as sim_data_file:
        csv_reader = csv.DictReader(sim_data_file)

        last_rep = -1

        for row in csv_reader:
            last_rep = int(row['repetition'])

    with open('powerbuoy_optimization_500_iter.csv', 'a') as sim_data_file:
        csv_writer = csv.writer(sim_data_file)

        for iteration_data in simulation_data:
            iteration_data[0] = last_rep + 1
            csv_writer.writerow(iteration_data)

