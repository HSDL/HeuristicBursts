import numpy
import copy
import heuristic_bursts.solution


class Agent(object):

    def __init__(self, options):
        self.options = options
        self.current_solution = heuristic_bursts.solution.Solution()
        self.team_current_solutions = []
        self.metrics = []
        self.weights = [1/3., 1/3., 1/3.]
        self.current_solution_quality = numpy.Inf
        if options.learning_style is 'multinomial':
            self.highertier_weights = numpy.ones(heuristic_bursts.solution.Solution.number_of_highertier_operations)
        elif options.learning_style is 'markovian':
            self.highertier_weights = numpy.ones([heuristic_bursts.solution.Solution.number_of_highertier_operations,
                                                  heuristic_bursts.solution.Solution.number_of_highertier_operations])

    def iterate(self):
        # Deep copy the candidate solution
        candidate_solution = copy.deepcopy(self.current_solution)

        # Decide which action to take

        # Apply that action and evaluate the outcome
        candidate_solution.evaluate()

        # Decide whether to keep the current solution or revert.

        # Tune approach based on evaluation and decision.

    def evaluate(self):
        self.metrics = self.current_solution.evaluate()
        self.current_solution_quality = [self.metrics[i]*self.weights[i] for i in range(heuristic_bursts.solution.Solution.number_of_metrics)]

    def interact(self):
        asdf = 1
        # Look at other solutions and see what you like
