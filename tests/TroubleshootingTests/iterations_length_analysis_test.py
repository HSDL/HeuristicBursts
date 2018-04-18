import csv
import numpy

import matplotlib
import matplotlib.pyplot as plt

num_iter = 300
max_allowed_qual = 750

with open('lower_tier_only_300_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    valid_reps = []
    for row in csv_reader:
        if int(row['iteration']) == num_iter and float(row['current solution quality']) < max_allowed_qual:
            valid_reps.append(row['repetition'])

    print('')
    print(valid_reps)
    print('')
    print(len(valid_reps))
    print('')

    sim_data_file.seek(0)

    next(csv_reader)

    iter_qual = []

    for i in range(0, num_iter):
        iter_qual.append([])

    for row in csv_reader:
        if row['repetition'] in valid_reps:
            iter_qual[int(row['iteration'])-1].append(float(row['current solution quality']))

    avg_iter_qual = []

    for it in iter_qual:
        avg_iter_qual.append(numpy.mean(it))

    print(avg_iter_qual)

plt.figure(1)
plt.subplot(111)
plt.ylabel('Average Solution Quality [W/kg]')
plt.title('Rule Effectiveness Using Only Lower-Tier Rules (300 Iterations per Agent)')
plt.plot(avg_iter_qual)

plt.show()
