from solution import Solution
import numpy as np


class Agent(object):

    def __init__(self, idx):
        self.current_solution = Solution()
        self.metrics = []
        self.weights = [1/3., 1/3., 1/3.]
        self.current_solution_quality = np.Inf

    def iterate(self):
        asdf = 1

    def evaluate(self):
        self.metrics = self.current_solution.evaluate()
        for i in range(Solution.number_of_metrics):
            self.quality = self.metrics[i]*self.weights[i]




