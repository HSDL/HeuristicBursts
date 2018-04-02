import numpy
import copy
import heuristic_bursts.solution
import random


class Agent(object):
    def __init__(self, options):
        # Setup agent according to options
        self.options = options

        # Instantiate solution class and evaluate initial solution
        self.current_solution = heuristic_bursts.solution.Solution()
        current_solution_for_evaluation = copy.deepcopy(self.current_solution)
        self.current_results = current_solution_for_evaluation.evaluate()
        self.all_solution_qualities = []

        # Instantiate lists for team solutions
        self.team_current_solutions = []
        self.team_current_results = []
        self.team_current_qualities = []
        self.team_solution_weights = []
        self.team_all_qualities = []

        # Set weights for evaluation metrics
        self.metrics = []
        self.weights = [1/2, 1/2, 0]
        self.evaluation_adjustments = [-1, 1, 0]
        self.current_solution_quality = numpy.Inf
        self.candidate_solution_quality = numpy.Inf

        # Determine agent learning style
        if options.learning_style is 'multinomial':
            self.highertier_weights = numpy.ones(heuristic_bursts.solution.Solution.number_of_highertier_operations)
        elif options.learning_style is 'markovian':
            self.highertier_weights = numpy.ones([heuristic_bursts.solution.Solution.number_of_highertier_operations,
                                                  heuristic_bursts.solution.Solution.number_of_highertier_operations])

        # Instantiate weights for different rule tiers
        self.preferred_rule_tier = 'low'
        self.tiers = ['low', 'high']
        self.tier_weights = [1.0, 0.0]
        self.tier_weight_change = [0.0, 0.0]

        # Initialize simulated annealing values for agent
        # self.TrikiParameter = 2.91*pow(10, -1)
        self.TrikiParameter = 3.00 * pow(10, -3)
        # self.initial_temperature = 1.65*pow(10, -2)
        self.initial_temperature = 0.006
        self.temperature = self.initial_temperature
        self.past_candidates_results = []
        self.candidate_variance = 1

        # Initialize iteration count for individual agent
        self.iteration_count = 1

    def iterate(self):
        # Deep copy the candidate solution
        self.candidate_solution = copy.deepcopy(self.current_solution)

        # Decide which action to take
        apply_lowtier_rule = False
        apply_hightier_rule = False

        # Each tier has its own weight for selection
        self.preferred_rule_tier = str(numpy.random.choice(self.tiers, p=self.tier_weights))
        # print('')
        # print('Preference:', self.preferred_rule_tier)
        if self.preferred_rule_tier == 'low':
            apply_lowtier_rule = True
        elif self.preferred_rule_tier == 'high':
            apply_hightier_rule = True

        # Depending which tier the agent prefers in this iteration, implement one rule from that tier
        if apply_lowtier_rule:
            rule = self.candidate_solution.lowtier_rule_select()
            # print("Rule Selected: ", rule)
            self.candidate_solution.lowtier_rule_perform(rule)
        elif apply_hightier_rule:
            rule = self.candidate_solution.hightier_rule_select()
            # print("Rule Selected: ", rule)
            self.candidate_solution.hightier_rule_perform(rule)

        # Deep copy the candidate to evaluate. Copying allows the candidate's position to be 'reset' after evalutation
        self.candidate_solution_for_evaluation = copy.deepcopy(self.candidate_solution)

        # Apply that action and evaluate the outcome
        if self.candidate_solution.is_valid():
            # self.candidate_solution_for_evaluation.display_visual = True
            self.candidate_results = self.candidate_solution_for_evaluation.evaluate()
            if self.candidate_solution_for_evaluation.stable_system:
                # Decide whether to keep the current solution or revert.
                # Compare quality of candidate solution to current solution
                self.evaluate()
                self.past_candidates_results.append(self.candidate_solution_quality)

        self.iteration_count += 1
        self.tier_weights = (self.tier_weights[0] + self.tier_weight_change[0],
                             self.tier_weights[1] + self.tier_weight_change[1])

        # Tune approach based on evaluation and decision.

    def compare_solutions(self, solution_a_results, solution_b_results):
        weighted_metric_comparison = [pow((solution_b_results[i]/solution_a_results[i]), self.comparison_inversions[i])
                                     *self.weights[i] for i in range(heuristic_bursts.solution.Solution.number_of_metrics)]
        # print('')
        # print("Comparison:", weighted_metric_comparison)
        result_ratio = sum(weighted_metric_comparison)
        return result_ratio

    def evaluate(self):
        # Calculate solution quality of current solution and candidate solution
        # self.current_solution_quality = sum([self.current_results[i] * self.weights[i] * self.evaluation_adjustments[i] for i in
        #                                  range(heuristic_bursts.solution.Solution.number_of_metrics)])
        self.current_solution_quality = self.current_results[1]/self.current_results[0]
        # self.candidate_solution_quality = sum([self.candidate_results[i] * self.weights[i] * self.evaluation_adjustments[i] for i in
        #                                  range(heuristic_bursts.solution.Solution.number_of_metrics)])
        self.candidate_solution_quality = self.candidate_results[1]/self.candidate_results[0]
        print("current quality:", self.current_solution_quality)
        print("candidate quality:", self.candidate_solution_quality)
        print('')

        # If quality shows candidate is better, accept candidate
        if self.candidate_solution_quality > self.current_solution_quality:
            self.current_solution = copy.deepcopy(self.candidate_solution)
            self.current_results = self.candidate_results
            self.current_solution_quality = self.candidate_solution_quality

        # If quality shows candidate is worse, probabilistically accept
        else:
            probability = numpy.exp((self.candidate_solution_quality - self.current_solution_quality)/self.temperature)
            random_num = random.random()
            if random_num <= probability:
                self.current_solution = copy.deepcopy(self.candidate_solution)
                self.current_results = self.candidate_results
                self.current_solution_quality = self.candidate_solution_quality
            print('probability:', probability)
            print('temp:', self.temperature)
            print('')

        self.all_solution_qualities.append(self.current_solution_quality)

        # Update temperature every 10th iteration
        if self.iteration_count % 10 == 0:
            self.update_temp()

    def update_temp(self):
        self.candidate_variance = numpy.var(self.past_candidates_results)
        self.temperature = self.temperature*(1-(self.temperature*self.TrikiParameter)/self.candidate_variance)
        self.past_candidates_results = []

    def interact(self):

        # Compare each solution to worst
        self.team_solution_weights = []
        for solution_quality in self.team_current_qualities:
            weight = solution_quality - min(self.team_current_qualities)
            self.team_solution_weights.append(weight)

        # Calculate probability of selecting each team solution
        team_solution_probabilities = []
        for i in range(0, len(self.team_solution_weights)):
            team_solution_probabilities.append(self.team_solution_weights[i]/sum(self.team_solution_weights))

        # Select which team solution to use starting in next iteration
        self.indices = []
        for i in range(0, len(self.team_current_solutions)):
            self.indices.append(i)
        solution_selected = numpy.random.choice(self.indices, p=team_solution_probabilities)
        self.current_solution = copy.deepcopy(self.team_current_solutions[solution_selected])
        self.current_results = self.team_current_results[solution_selected]
        print(self.current_results)
        self.all_solution_qualities = self.team_all_qualities[solution_selected]
        # self.current_solution_quality = sum([self.current_results[i] * self.weights[i] * self.evaluation_adjustments[i]
        #                                      for i in range(heuristic_bursts.solution.Solution.number_of_metrics)])
        self.current_solution_quality = self.current_results[1]/self.current_results[0]
