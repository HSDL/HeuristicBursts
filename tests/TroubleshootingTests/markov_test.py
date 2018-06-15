import heuristic_bursts.agent
import heuristic_bursts.options

import wec.wec
import wec.wec_visual
import time

import numpy

options = heuristic_bursts.options.Options()
agent = heuristic_bursts.agent.Agent(options)

print('Markov:', agent.markov_matrix)
print('Normal Markov:', agent.markov_normal)
print('')

for i in range(0, 50):
    agent.iterate()
    print('Tier:', agent.preferred_rule_tier)
    print('Rule:', agent.rule)
    print('Markov:', agent.markov_matrix)
    print('Normal Markov:', agent.markov_normal)
    print('')
