import csv
import numpy

import matplotlib
import matplotlib.pyplot as plt
from scipy import stats

num_iter = 500
num_reps = 75
max_allowed_qual = 3000

with open('markov_test_500_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual and len(valid_reps) < num_reps:
            valid_reps.append(row['repetition'])

    print('')
    print(valid_reps)
    print('')
    print(len(valid_reps))
    print('')

    sim_data_file.seek(0)

    next(csv_reader)

    iter_qual_a = []

    for i in range(0, num_iter):
        iter_qual_a.append([])

    for row in csv_reader:
        if row['repetition'] in valid_reps:
            iter_qual_a[int(row['iteration'])-1].append(float(row['current solution quality']))

    avg_iter_qual_a = []

    for it in iter_qual_a:
        avg_iter_qual_a.append(numpy.mean(it))

    print(avg_iter_qual_a)

    iter_err_a = []

    for it in iter_qual_a:
        iter_err_a.append(stats.sem(it))

    print(iter_err_a)

with open('markov_test_lower_only_500_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual and len(valid_reps) < num_reps:
            valid_reps.append(row['repetition'])

    print('')
    print(valid_reps)
    print('')
    print(len(valid_reps))
    print('')

    sim_data_file.seek(0)

    next(csv_reader)

    iter_qual_b = []

    for i in range(0, num_iter):
        iter_qual_b.append([])

    for row in csv_reader:
        if row['repetition'] in valid_reps:
            iter_qual_b[int(row['iteration'])-1].append(float(row['current solution quality']))

    avg_iter_qual_b = []

    for it in iter_qual_b:
        avg_iter_qual_b.append(numpy.mean(it))

    print(avg_iter_qual_b)

    iter_err_b = []

    for it in iter_qual_b:
        iter_err_b.append(stats.sem(it))

    print(iter_err_b)

with open('markov_test_higher_only_500_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual and len(
                valid_reps) < num_reps:
            valid_reps.append(row['repetition'])

    print('')
    print(valid_reps)
    print('')
    print(len(valid_reps))
    print('')

    sim_data_file.seek(0)

    next(csv_reader)

    iter_qual_c = []

    for i in range(0, num_iter):
        iter_qual_c.append([])

    for row in csv_reader:
        if row['repetition'] in valid_reps:
            iter_qual_c[int(row['iteration']) - 1].append(float(row['current solution quality']))

    avg_iter_qual_c = []

    for it in iter_qual_c:
        avg_iter_qual_c.append(numpy.mean(it))

    print(avg_iter_qual_c)

    iter_err_c = []

    for it in iter_qual_c:
        iter_err_c.append(stats.sem(it))

    print(iter_err_c)

with open('markov_test_preloaded_matrix_500_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual and len(
                valid_reps) < num_reps:
            valid_reps.append(row['repetition'])

    print('')
    print(valid_reps)
    print('')
    print(len(valid_reps))
    print('')

    sim_data_file.seek(0)

    next(csv_reader)

    iter_qual_d = []

    for i in range(0, num_iter):
        iter_qual_d.append([])

    for row in csv_reader:
        if row['repetition'] in valid_reps:
            iter_qual_d[int(row['iteration']) - 1].append(float(row['current solution quality']))

    avg_iter_qual_d = []

    for it in iter_qual_d:
        avg_iter_qual_d.append(numpy.mean(it))

    print(avg_iter_qual_d)

    iter_err_d = []

    for it in iter_qual_d:
        iter_err_d.append(stats.sem(it))

    print(iter_err_d)


plt.figure(1)
plt.subplot(111)
plt.xlabel('Iteration')
plt.ylabel('Average Solution Quality [W/kg]')
plt.title('Rule Effectiveness Using Markov Chain-Based Rule Applications (500 Iterations per Agent)')
plt.grid(color='g', linestyle='-', linewidth=0.25)
plt.xlim((0, 500))
# plt.legend((avg_iter_qual_a, avg_iter_qual_b, avg_iter_qual_c),("Lower-Tier Only", "Higher-Tier Only", "Lower-Tier Burst"), loc=4)
# plt.plot(avg_iter_qual)
plt.errorbar(numpy.arange(500), avg_iter_qual_a, yerr=iter_err_a, color='b', ecolor='b', elinewidth=0.25, label="Markov w/ Both Tiers")
plt.errorbar(numpy.arange(500), avg_iter_qual_b, yerr=iter_err_b, color='g', ecolor='g', elinewidth=0.25, label="Markov w/ Lower Tier")
plt.errorbar(numpy.arange(500), avg_iter_qual_c, yerr=iter_err_c, color='m', ecolor='m', elinewidth=0.25, label="Markov w/ Higher Tier")
plt.errorbar(numpy.arange(500), avg_iter_qual_d, yerr=iter_err_d, color='y', ecolor='y', elinewidth=0.25, label="Markov w/ Preloaded Average Matrix")
plt.legend(loc=4)


labels = ['Lower-Tier Only', 'Higher-Tier Only', 'Lower-Tier Burst']
final_quals = [avg_iter_qual_a[-1], avg_iter_qual_b[-1], avg_iter_qual_c[-1]]
final_errs = [iter_err_a[-1], iter_err_b[-1], iter_err_c[-1]]

# plt.figure(2)
# plt.subplot(211)
# plt.xticks(range(len(final_quals)), labels)
# plt.ylabel('Solution Quality [W/kg]')
# plt.title('Average Final Solution Quality for Each Rule-Tier Preference Setting (After 500 Iterations)')
# plt.grid(axis='y', color='b', linestyle='-', linewidth=0.25)
# plt.bar(range(len(final_quals)), final_quals, 1/1.5, align='center', color='g')
# plt.errorbar(range(len(final_quals)), final_quals, yerr=final_errs, fmt='none', ecolor='k', capsize=10)
#
# early_quals = [avg_iter_qual_a[100], avg_iter_qual_b[100], avg_iter_qual_c[100]]
# early_errs = [iter_err_a[100], iter_err_b[100], iter_err_c[100]]
#
# plt.figure(3)
# plt.subplot(311)
# plt.xticks(range(len(final_quals)), labels)
# plt.ylabel('Solution Quality [W/kg]')
# plt.title('Average Solution Quality for Each Rule-Tier Preference Setting (After 100 Iterations)')
# plt.grid(axis='y', color='b', linestyle='-', linewidth=0.25)
# plt.bar(range(len(early_quals)), early_quals, 1/1.5, align='center', color='g')
# plt.errorbar(range(len(early_quals)), early_quals, yerr=early_errs, fmt='none', ecolor='k', capsize=10)

plt.show()
