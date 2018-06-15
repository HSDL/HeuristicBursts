import heuristic_bursts.agent
import heuristic_bursts.team
import heuristic_bursts.options

import wec.wec
import wec.wec_visual
import time

import csv
import numpy
import copy

num_iter = 500
num_reps = 75
max_allowed_qual = 3000

with open('markov_test_500_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual and len(valid_reps) < num_reps:
            valid_reps.append(row['repetition'])

    sim_data_file.seek(0)

    next(csv_reader)

    all_final_markov_matrices = []

    for row in csv_reader:
        if row['repetition'] in valid_reps and int(row['iteration']) == num_iter:
            markov_matrix_str = row['Markov matrix']
            markov_matrix_str = markov_matrix_str[1:]
            markov_matrix_str = markov_matrix_str[:-1]

            markov_matrix = []
            for item in markov_matrix_str:
                if item == '[':
                    next_row = []
                    next_num = []
                elif item == ']' and len(next_num) > 0:
                    next_num = ''.join(next_num)
                    next_row.append(float(next_num))
                    next_num = []
                    markov_matrix.append(next_row)
                    next_row = []
                elif item == ']' and len(next_row) > 0:
                    markov_matrix.append(next_row)
                    next_row = []
                elif item == ' ' and len(next_num) > 0:
                    next_num = ''.join(next_num)
                    next_row.append(float(next_num))
                    next_num = []
                else:
                    if item != ' ' and item != '\n':
                        next_num.append(item)

            all_final_markov_matrices.append(numpy.array(markov_matrix))

avg_markov_matrix = numpy.mean(all_final_markov_matrices, axis=0)

options = heuristic_bursts.options.Options()

# with open('markov_test_preloaded_matrix_500_iterations.csv', 'w') as sim_data_file:
#     fieldnames = ['repetition', 'iteration', 'rule tier', 'rule number', 'quality before rule', 'quality after rule',
#                   'current solution quality', 'rule acceptance', 'lower tier preference', 'higher tier preference',
#                   'error', 'Markov matrix']
#
#     csv_writer = csv.DictWriter(sim_data_file, fieldnames=fieldnames)
#     csv_writer.writeheader()

for sim_num in range(0, 50):

    # Instantiate team
    team = heuristic_bursts.team.Team(options)

    # Set team preloaded average markov matrix
    for agent in team.agent_list:
        agent.markov_matrix = copy.deepcopy(avg_markov_matrix)

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

    with open('markov_test_preloaded_matrix_500_iterations.csv', 'r') as sim_data_file:
        csv_reader = csv.DictReader(sim_data_file)

        last_rep = -1

        for row in csv_reader:
            last_rep = int(row['repetition'])

    with open('markov_test_preloaded_matrix_500_iterations.csv', 'a') as sim_data_file:
        csv_writer = csv.writer(sim_data_file)

        for iteration_data in simulation_data:
            iteration_data[0] = last_rep + 1
            csv_writer.writerow(iteration_data)
