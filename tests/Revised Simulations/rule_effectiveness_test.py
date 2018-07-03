import csv
import numpy

import matplotlib
import matplotlib.pyplot as plt

num_iter = 400
max_allowed_qual = 10000


with open('lower_tier_only_400_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual:
            valid_reps.append(row['repetition'])

    print('')
    print(valid_reps)
    print('')

    sim_data_file.seek(0)

    next(csv_reader)

    rule_qual = []

    for i in range(0, 8):
        rule_qual.append([])

    for row in csv_reader:
        if row['repetition'] in valid_reps and int(row['rule acceptance']) == 1 and int(row['iteration']):
            qual_diff = float(row['quality after rule']) - float(row['quality before rule'])
            rule_qual[int(row['rule number'])-1].append(qual_diff)

    avg_rule_qual = []

    for rule in rule_qual:
        avg_rule_qual.append(numpy.mean(rule))

    print(avg_rule_qual)

plt.figure(1)
plt.subplot(111)
rule_list = ['L1: Add rotational body', 'L2: Add linear body', 'L3: Remove body with joint', 'L4: Change joint type',
             'L5: Change body dimensions', 'L6: Relocate body with joint', 'L7: Swap bodies',
             'L8: Change joint coefficients']
plt.xlabel('Average Solution Quality Improvement [W/kg]')
plt.yticks(range(len(avg_rule_qual)), rule_list)
plt.title('Average Rule Effectiveness Using Only Lower-Tier Rules (Rules Accepted in Design)(400 Iterations per Agent)(All Iterations)')
plt.grid(axis='x', color='r', linestyle='-', linewidth=1)
plt.xlim((0, 50))
plt.barh(range(len(avg_rule_qual)), avg_rule_qual, 1/1.5, align='center')
plt.show()

with open('lower_tier_only_400_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual:
            valid_reps.append(row['repetition'])

    print('')
    print(valid_reps)
    print('')

    sim_data_file.seek(0)

    next(csv_reader)

    rule_qual = []

    for i in range(0, 8):
        rule_qual.append([])

    for row in csv_reader:
        if row['repetition'] in valid_reps and int(row['rule acceptance']) == 1 and int(row['iteration']) <= 200:
            qual_diff = float(row['quality after rule']) - float(row['quality before rule'])
            rule_qual[int(row['rule number'])-1].append(qual_diff)

    avg_rule_qual = []

    for rule in rule_qual:
        avg_rule_qual.append(numpy.mean(rule))

    print(avg_rule_qual)

plt.figure(1)
plt.subplot(111)
rule_list = ['L1: Add rotational body', 'L2: Add linear body', 'L3: Remove body with joint', 'L4: Change joint type',
             'L5: Change body dimensions', 'L6: Relocate body with joint', 'L7: Swap bodies',
             'L8: Change joint coefficients']
plt.xlabel('Average Solution Quality Improvement [W/kg]')
plt.yticks(range(len(avg_rule_qual)), rule_list)
plt.title('Average Rule Effectiveness Using Only Lower-Tier Rules (Rules Accepted in Design)(400 Iterations per Agent)(First 200 Iterations)')
plt.grid(axis='x', color='r', linestyle='-', linewidth=1)
plt.xlim((0, 50))
plt.barh(range(len(avg_rule_qual)), avg_rule_qual, 1/1.5, align='center')
plt.show()

with open('lower_tier_only_400_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual:
            valid_reps.append(row['repetition'])

    print('')
    print(valid_reps)
    print('')

    sim_data_file.seek(0)

    next(csv_reader)

    rule_qual = []

    for i in range(0, 8):
        rule_qual.append([])

    for row in csv_reader:
        if row['repetition'] in valid_reps and int(row['rule acceptance']) == 1 and int(row['iteration']) > 200:
            qual_diff = float(row['quality after rule']) - float(row['quality before rule'])
            rule_qual[int(row['rule number'])-1].append(qual_diff)

    avg_rule_qual = []

    for rule in rule_qual:
        avg_rule_qual.append(numpy.mean(rule))

    print(avg_rule_qual)

plt.figure(1)
plt.subplot(111)
rule_list = ['L1: Add rotational body', 'L2: Add linear body', 'L3: Remove body with joint', 'L4: Change joint type',
             'L5: Change body dimensions', 'L6: Relocate body with joint', 'L7: Swap bodies',
             'L8: Change joint coefficients']
plt.xlabel('Average Solution Quality Improvement [W/kg]')
plt.yticks(range(len(avg_rule_qual)), rule_list)
plt.title('Average Rule Effectiveness Using Only Lower-Tier Rules (Rules Accepted in Design)(400 Iterations per Agent)(Second 200 Iterations)')
plt.grid(axis='x', color='r', linestyle='-', linewidth=1)
plt.xlim((0, 50))
plt.barh(range(len(avg_rule_qual)), avg_rule_qual, 1/1.5, align='center')
plt.show()

# with open('lower_tier_only_400_iterations.csv', 'r') as sim_data_file:
#     csv_reader = csv.DictReader(sim_data_file)
#
#     valid_reps = []
#     for row in csv_reader:
#         if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual:
#             valid_reps.append(row['repetition'])
#
#     print('')
#     print(valid_reps)
#     print('')
#
#     sim_data_file.seek(0)
#
#     next(csv_reader)
#
#     rule_qual = []
#
#     for i in range(0, 8):
#         rule_qual.append([])
#
#     for row in csv_reader:
#         if row['repetition'] in valid_reps:
#             qual_diff = float(row['quality after rule']) - float(row['quality before rule'])
#             rule_qual[int(row['rule number'])-1].append(qual_diff)
#
#     avg_rule_qual = []
#
#     for rule in rule_qual:
#         avg_rule_qual.append(numpy.mean(rule))
#
#     print(avg_rule_qual)
#
# plt.figure(1)
# plt.subplot(111)
# rule_list = ['L1: Add rotational body', 'L2: Add linear body', 'L3: Remove body with joint', 'L4: Change joint type',
#              'L5: Change body dimensions', 'L6: Relocate body with joint', 'L7: Swap bodies',
#              'L8: Change joint coefficients']
# plt.xlabel('Average Solution Quality Improvement [W/kg]')
# plt.yticks(range(len(avg_rule_qual)), rule_list)
# plt.title('Average Rule Effectiveness Using Only Lower-Tier Rules (Rules Accepted in Design)(300 Iterations per Agent)')
# plt.barh(range(len(avg_rule_qual)), avg_rule_qual, 1/1.5, align='center')
# plt.show()