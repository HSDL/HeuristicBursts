import csv
import numpy

import matplotlib
import matplotlib.pyplot as plt

rep_num = 28

with open('lower_tier_only_300_iterations.csv', 'r') as sim_data_file:
    csv_reader = csv.DictReader(sim_data_file)

    rep_qualities = []
    for row in csv_reader:
        last_rep = row['repetition']
        if int(row['repetition']) == rep_num:
            rep_qualities.append(float(row['current solution quality']))

    print(len(rep_qualities))
    print(last_rep)

plt.figure(1)
plt.subplot(111)
plt.ylabel('Solution Quality [W/kg]')
plt.title('Rule Effectiveness Using Only Lower-Tier Rules (300 Iterations per Agent)')
plt.plot(rep_qualities)

plt.show()
