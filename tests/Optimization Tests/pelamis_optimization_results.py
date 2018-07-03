import csv
import numpy

import matplotlib
import matplotlib.pyplot as plt

rep_num = 0
num_iter = 500

with open('pelamis_optimization_500_iter.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    rep_qualities = []
    for row in csv_reader:
        last_rep = row['repetition']
        if int(row['repetition']) == rep_num:
            rep_qualities.append(float(row['current solution quality']))

    print(len(rep_qualities))
    print(last_rep)

    sim_data_file.seek(0)

    next(csv_reader)

    iter_qual = []

    for i in range(0, num_iter):
        iter_qual.append([])

    for row in csv_reader:
        iter_qual[int(row['iteration']) - 1].append(float(row['current solution quality']))

    avg_iter_qual = []

    for it in iter_qual:
        avg_iter_qual.append(numpy.mean(it))

plt.figure(1)
plt.subplot(111)
plt.ylabel('Solution Quality [W/kg]')
plt.title('Rule Effectiveness Using Only Lower-Tier Rules (300 Iterations per Agent)')
plt.plot(rep_qualities)

plt.show()

plt.figure(2)
plt.subplot(111)
plt.xlabel('Agent Iteration')
plt.ylabel('Average Solution Quality [W/kg]')
plt.title('Optimization of Pelamis WEC Device')
plt.plot(avg_iter_qual)

plt.show()
