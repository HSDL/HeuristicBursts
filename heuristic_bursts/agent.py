import numpy as np
import heuristic_bursts


class Agent(object):

    def __init__(self, options):
        self.options = options
        self.current_solution = heuristic_bursts.solution.Solution()
        self.metrics = []
        self.weights = [1/3., 1/3., 1/3.]
        self.current_solution_quality = np.Inf

    def iterate(self):
        # First, decide what kind of action to do
        # Second, apply the rule:
        # Third, evaluate: self.candidate_solution.evaluate()
        # Fourth, tune approach based on evaluation.
        asdf = 1

    def evaluate(self):
        self.metrics = self.current_solution.evaluate()
        self.current_solution_quality = [self.metrics[i]*self.weights[i] for i in range(heuristic_bursts.solution.Solution.number_of_metrics)]
