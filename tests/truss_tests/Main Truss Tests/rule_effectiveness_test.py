import csv
import numpy

import matplotlib
import matplotlib.pyplot as plt
from scipy import stats

num_iter = 500
num_reps = 100

file = 'lower_tier_only_500_iterations.csv'

all_repetitions_data = []

num_lowtier_rules = 7
num_hightier_rules = 0

chunk_size = 20
num_chunks = num_iter/chunk_size

rule_applications_a = []
rule_acceptance_a = []
rule_effectiveness_a = []
rule_selection_chance_a = []
rule_proportions_a = []

for i in range(0, int(num_chunks)):
    rule_applications_a.append(numpy.zeros(num_lowtier_rules+num_hightier_rules))
    rule_acceptance_a.append(numpy.zeros(num_lowtier_rules+num_hightier_rules))
    rule_effectiveness_a.append(numpy.zeros(num_lowtier_rules+num_hightier_rules))

with open(file, 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and len(valid_reps) < num_reps:
            valid_reps.append(row['repetition'])

    print('')
    print(valid_reps)
    print('')
    print(len(valid_reps))
    print('')

    sim_data_file.seek(0)

    next(csv_reader)

    data_list = list(csv_reader)
    current_data_list_index = 0

    for repetition_index in range(0, len(valid_reps)):
        current_rep_num = valid_reps[repetition_index]
        current_rep_data = []

        for i in range(0, num_iter):
            current_rep_data.append([])

        for data_index in range(current_data_list_index, len(data_list)):
            row = data_list[data_index]
            current_data_list_index += 1
            if row['repetition'] == current_rep_num:
                rep = int(current_rep_num)
                iter = int(row['iteration'])
                tier = row['rule tier']
                rule = int(row['rule number'])
                acceptance = int(row['rule acceptance'])

                quality_before = float(row['quality before rule'])
                quality_after = float(row['quality after rule'])
                quality_change = quality_after - quality_before

                current_rep_data[int(row['iteration'])-1].append({'rep': rep,
                                                                  'iter': iter,
                                                                  'tier': tier,
                                                                  'rule': rule,
                                                                  'acceptance': acceptance,
                                                                  'quality_change': quality_change})

            elif row['repetition'] in valid_reps:
                current_data_list_index -= 1
                break
        # print(current_rep_data)
        all_repetitions_data.append(current_rep_data)

# print(all_repetitions_data)

for i in range(0, len(all_repetitions_data)):
    for j in range(0, len(all_repetitions_data[i])):
        iteration = all_repetitions_data[i][j][0]
        chunk = int((iteration['iter'] - 1) / chunk_size)

        if iteration['tier'] == 'low':
            rule_index = iteration['rule'] - 1
        elif iteration['tier'] == 'high':
            rule_index = iteration['rule'] - 1 + num_lowtier_rules

        rule_applications_a[chunk][rule_index] += 1

        if iteration['acceptance'] == 1:
            rule_acceptance_a[chunk][rule_index] += 1

rule_effectiveness_a = numpy.divide(rule_acceptance_a, rule_applications_a)
rule_selection_chance_a = numpy.divide(rule_applications_a, len(all_repetitions_data)*chunk_size)
print(rule_acceptance_a)

for i in range(0, len(rule_acceptance_a)):
    total_accepted = sum(rule_acceptance_a[i])
    rule_proportions_a.append(numpy.divide(rule_acceptance_a[i], total_accepted))

error_a = []

print(len(rule_proportions_a))

for chunk in rule_proportions_a:
    error_a.append(stats.sem(chunk))

# print(rule_applications_a)
# print(rule_acceptance_a)
# print(rule_effectiveness_a)
# print(rule_selection_chance_a)
# print(rule_proportions_a)
# print('')

# file = 'probabilistic_selection_test_1000_iterations.csv'
#
# all_repetitions_data = []
#
# rule_applications_b = []
# rule_acceptance_b = []
# rule_effectiveness_b = []
# rule_selection_chance_b = []
# rule_proportions_b = []
#
# for i in range(0, int(num_iter/chunk_size)):
#     rule_applications_b.append(numpy.zeros(num_lowtier_rules+num_hightier_rules))
#     rule_acceptance_b.append(numpy.zeros(num_lowtier_rules+num_hightier_rules))
#     rule_effectiveness_b.append(numpy.zeros(num_lowtier_rules+num_hightier_rules))
#
# with open(file, 'r') as sim_data_file:
#     csv_reader = csv.DictReader(sim_data_file)
#
#     valid_reps = []
#     for row in csv_reader:
#         if int(row['iteration']) == num_iter and len(valid_reps) < num_reps:
#             valid_reps.append(row['repetition'])
#
#     print('')
#     print(valid_reps)
#     print('')
#     print(len(valid_reps))
#     print('')
#
#     sim_data_file.seek(0)
#
#     next(csv_reader)
#
#     data_list = list(csv_reader)
#     current_data_list_index = 0
#
#     for repetition_index in range(0, len(valid_reps)):
#         current_rep_num = valid_reps[repetition_index]
#         current_rep_data = []
#
#         for i in range(0, num_iter):
#             current_rep_data.append([])
#
#         for data_index in range(current_data_list_index, len(data_list)):
#             row = data_list[data_index]
#             current_data_list_index += 1
#             if row['repetition'] == current_rep_num:
#                 rep = int(current_rep_num)
#                 iter = int(row['iteration'])
#                 tier = row['rule tier']
#                 rule = int(row['rule number'])
#                 acceptance = int(row['rule acceptance'])
#
#                 quality_before = float(row['quality before rule'])
#                 quality_after = float(row['quality after rule'])
#                 quality_change = quality_after - quality_before
#
#                 current_rep_data[int(row['iteration'])-1].append({'rep': rep,
#                                                                   'iter': iter,
#                                                                   'tier': tier,
#                                                                   'rule': rule,
#                                                                   'acceptance': acceptance,
#                                                                   'quality_change': quality_change})
#
#             elif row['repetition'] in valid_reps:
#                 current_data_list_index -= 1
#                 break
#
#         all_repetitions_data.append(current_rep_data)
#
# for i in range(0, len(all_repetitions_data)):
#     for j in range(0, len(all_repetitions_data[i])):
#         iteration = all_repetitions_data[i][j][0]
#         chunk = int((iteration['iter'] - 1) / chunk_size)
#
#         if iteration['tier'] == 'low':
#             rule_index = iteration['rule'] - 1
#         elif iteration['tier'] == 'high':
#             rule_index = iteration['rule'] - 2 + num_lowtier_rules
#
#         rule_applications_b[chunk][rule_index] += 1
#
#         if iteration['acceptance'] == 1:
#             rule_acceptance_b[chunk][rule_index] += 1
#
# rule_effectiveness_b = numpy.divide(rule_acceptance_b, rule_applications_b)
# rule_selection_chance_b = numpy.divide(rule_applications_b, len(all_repetitions_data)*chunk_size)
#
# for i in range(0, len(rule_acceptance_b)):
#     total_accepted = sum(rule_acceptance_b[i])
#     rule_proportions_b.append(numpy.divide(rule_acceptance_b[i], total_accepted))
#
# error_b = []
#
# for chunk in rule_proportions_b:
#     error_b.append(stats.sem(chunk))

# print(rule_applications_b)
# print(rule_acceptance_b)
# print(rule_effectiveness_b)

#######################################################################################################################
#######################################################################################################################

chunk_labels = ('1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '20')
y_pos = numpy.arange(num_chunks)
bar_width = 1

# for rule in range(0, num_lowtier_rules + num_hightier_rules):
#     effectiveness_a = []
#     effectiveness_b = []
#     for chunk in rule_proportions_a:
#         effectiveness_a.append(chunk[rule])
#     for chunk in rule_proportions_b:
#         effectiveness_b.append(chunk[rule])
#     plt.bar(y_pos, effectiveness_a, bar_width, color='g', align='center', alpha=0.5, label='Random Selection')
#     # plt.bar(y_pos+bar_width, effectiveness_b, bar_width, color='c', align='center', alpha=0.5, label='Probabilistic Selection')
#     # plt.errorbar(y_pos, effectiveness_a, yerr=error_a, color='g', alpha=0.5, fmt='o')
#     # plt.errorbar(y_pos + bar_width, effectiveness_b, yerr=error_b, color='c', alpha=0.5, fmt='o')
#     plt.xticks(y_pos, chunk_labels)
#     plt.ylim(0, 0.75)
#     plt.grid()
#     plt.xlabel('Iteration Chunk (Every 100 Iter.)')
#     plt.ylabel('Acceptance Rate of Applied Rule')
#     plt.legend(loc=1)
#
    # if rule < 8:
    #     plt.title('Lower-Tier Rule: ' + str(rule+1))
    # else:
    #     plt.title('Higher-Tier Rule: ' + str(rule-7))
#     print(effectiveness_a)
#     print(effectiveness_b)
#     plt.show()

all_rule_proportions = []

for rule in range(0, num_lowtier_rules + num_hightier_rules):
    proportion = []
    for chunk in rule_proportions_a:
        proportion.append(chunk[rule])
    all_rule_proportions.append(proportion)

print(all_rule_proportions)

colors = [(0.8, 0, 0), (0, 0.8, 0), (0, 0, 0.8), (0.8, 0.8, 0), (0.8, 0, 0.8), (0, 0.8, 0.8), (0.8, 0.4, 0.4), (0.4, 0.8, 0.4),
          (0.4, 0.4, 0.8), (0.8, 0.2, 0.4), (0.2, 0.2, 0), (0.8, 1.0, 0.4), (0.9, 0.6, 0.2)]
last_bottom = numpy.zeros(len(rule_proportions_a))

for rule_index in range(0, len(all_rule_proportions)):
    rule = all_rule_proportions[rule_index]
    if rule_index < 7:
        rule_name = "LT Rule: " + str(rule_index+1)
    else:
        rule_name = "HT Rule: "+str(rule_index-6)
    # all_rule_names = ["HT 1: Increase Complexity", "HT 2: Decrease Complexity", "HT 3: Change Scale", "HT 4: Replicate Pattern", "HT 5: Standardize"]
    all_rule_names = ["LT 1: Split Member", "LT 2: Join Member", "LT 3: Add Joint", "LT 4: Remove Joint", "LT 5: Switch Diagonal Member", "LT 6: Move Joint", "LT 7: Re-Size Member"]
    rule_name = all_rule_names[rule_index]
    plt.bar(y_pos, rule, bar_width, color=colors[rule_index], bottom=last_bottom, align='center', alpha=0.5, label=rule_name)
    plt.xticks(y_pos, chunk_labels)
    plt.xlim(-0.5, 9.5)
    plt.ylim(0, 1.0)
    plt.xlabel('Iteration Chunk (Every 20 Iter.)')
    plt.ylabel('Proportion')
    plt.title('Proportion of Each Rule Within All Accepted Rules per Chunk (Lower Tier Only)')
    plt.legend(loc=0)

    last_bottom += rule

# plt.grid()
plt.show()

#######################################################################################################################
#######################################################################################################################

# lumped_proportions = numpy.zeros(int(num_chunks))
# best_rules = [3, 5, 6, 9, 11]
#
# for rule_index in range(0, len(all_rule_proportions)):
#     if rule_index not in best_rules:
#         for chunk_index in range(len(rule_proportions_a)):
#             lumped_proportions[chunk_index] += all_rule_proportions[rule_index][chunk_index]
#     print(lumped_proportions)
#
# lumped_and_best_proportions = []
# lumped_and_best_proportions.append(lumped_proportions)
#
# for index in best_rules:
#     lumped_and_best_proportions.append(all_rule_proportions[index])
#
# colors = [(0.5, 0.4, 0.2), (0, 0.6, 0), (0, 0.2, 0.5), (0.7, 0.2, 0.1), (0.4, 0.8, 0.2), (0.8, 0.5, 0)]
# last_bottom = numpy.zeros(len(rule_proportions_a))
#
# for rule_index in range(0, len(lumped_and_best_proportions)):
#     rule = lumped_and_best_proportions[rule_index]
#     if rule_index == 0:
#         rule_name = "OTHER"
#     elif rule_index == 1:
#         rule_name = 'LT Rule 4'
#     elif rule_index == 2:
#         rule_name = 'LT Rule 6'
#     elif rule_index == 3:
#         rule_name = 'LT Rule 7'
#     elif rule_index == 4:
#         rule_name = 'HT Rule 3'
#     elif rule_index == 5:
#         rule_name = 'HT Rule 5'
#     plt.bar(y_pos, rule, bar_width, color=colors[rule_index], bottom=last_bottom, align='center', alpha=0.5,
#             label=rule_name)
#     plt.xticks(y_pos, chunk_labels)
#     plt.xlim(-0.5, 19.5)
#     plt.ylim(0, 1.0)
#     plt.xlabel('Iteration Chunk (Every 20 Iter.)')
#     plt.ylabel('Proportion')
#     plt.title('Proportion of Each Rule Within All Accepted Rules per Chunk (Both Tiers w/ Random Selection)')
#     plt.legend(loc=1)
#
#     last_bottom += rule
#
# # plt.grid()
# plt.show()

#######################################################################################################################
#######################################################################################################################

# lower_tier_proportions = numpy.zeros(int(num_chunks))
# higher_tier_proportions = numpy.zeros(int(num_chunks))
#
# for rule_index in range(len(all_rule_proportions)):
#     if rule_index < 8:
#         for chunk_index in range(len(all_rule_proportions[rule_index])):
#             lower_tier_proportions[chunk_index] += all_rule_proportions[rule_index][chunk_index]
#         print(lower_tier_proportions)
#     elif rule_index >= 8:
#         for chunk_index in range(len(all_rule_proportions[rule_index])):
#             higher_tier_proportions[chunk_index] += all_rule_proportions[rule_index][chunk_index]
#         print(higher_tier_proportions)
#
# combined_tiers_proportions = []
# combined_tiers_proportions.append(lower_tier_proportions)
# combined_tiers_proportions.append(higher_tier_proportions)
#
# colors = [(0.8, 0.8, 0), (0, 0.2, 0.8)]
#
# last_bottom = numpy.zeros(len(rule_proportions_a))
#
# for tier_index in range(len(combined_tiers_proportions)):
#     tier = combined_tiers_proportions[tier_index]
#     if tier_index == 0:
#         tier_name = "Lower-Tier"
#     elif tier_index == 1:
#         tier_name = "Higher-Tier"
#     plt.bar(y_pos, tier, bar_width, color=colors[tier_index], bottom=last_bottom, align='center', alpha=0.5, label=tier_name)
#     plt.xticks(y_pos, chunk_labels)
#     # plt.xticks([])
#     plt.ylim(0, 1.0)
#     plt.xlabel('Iteration Chunk (Every 25 Iter.)')
#     plt.ylabel('Proportion')
#     plt.title('Proportion of Each Rule Tier Within All Accepted Rules per Chunk (Both Tiers w/ Random Selection)(500 Iterations per Agent)')
#     plt.legend(loc=1)
#
#     last_bottom += tier
#
# plt.grid(axis='y', linestyle='-')
# plt.show()
