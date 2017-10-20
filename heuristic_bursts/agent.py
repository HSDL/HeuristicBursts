import numpy as np
from heuristic_bursts.solution import Solution


class Agent(object):

    def __init__(self):
        self.current_solution = Solution()
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
        for i in range(Solution.number_of_metrics):
            self.quality = self.metrics[i]*self.weights[i]




