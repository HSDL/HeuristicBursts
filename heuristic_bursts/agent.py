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
        self.team_all_simulation_data = []

        # Set weights for evaluation metrics
        self.metrics = []
        self.weights = [1/2, 1/2, 0]
        self.evaluation_adjustments = [-1, 1, 0]
        self.current_solution_quality = 0.0
        self.candidate_solution_quality = 0.0
        self.previous_solution_quality = self.current_solution_quality

        # Determine agent learning style
        if options.learning_style is 'multinomial':
            self.highertier_weights = numpy.ones(heuristic_bursts.solution.Solution.number_of_highertier_operations)
        elif options.learning_style is 'markovian':
            self.highertier_weights = numpy.ones([heuristic_bursts.solution.Solution.number_of_highertier_operations,
                                                  heuristic_bursts.solution.Solution.number_of_highertier_operations])

        # Instantiate weights for different rule tiers
        self.rule = 1
        self.last_rule = 1
        self.preferred_rule_tier = 'low'
        self.last_tier = 'low'
        self.apply_lowtier_rule = False
        self.apply_hightier_rule = False
        self.tiers = ['low', 'high']
        self.tier_weights = [1.0, 0.0]
        self.tier_weight_change = [0.0, 0.0]
        self.rule_accepted = 0

        # Initialize simulated annealing values for agent
        # self.TrikiParameter = 2.91*pow(10, -1)
        self.TrikiParameter = 2000
        # self.initial_temperature = 1.65*pow(10, -2)
        self.initial_temperature = 500
        self.temperature = self.initial_temperature
        self.past_candidates_results = []
        self.candidate_variance = 1

        # Initialize matrices for previous-tier weighted rule selection
        self.previous_tier_preferred_k = 0.1
        self.previous_tier_preferred_selection_weights = [[0.9, 0.1], [0.1, 0.9]]

        # Initialize Markov matrices
        self.markov_k = 0.05
        self.num_lowtier_rules = 7
        self.num_hightier_rules = 5
        self.markov_matrix = numpy.ones((self.num_lowtier_rules + self.num_hightier_rules, self.num_lowtier_rules + self.num_hightier_rules))
        self.markov_normal = numpy.ones((self.num_lowtier_rules + self.num_hightier_rules, self.num_lowtier_rules + self.num_hightier_rules))
        self.normalize_markov()

        # Initialize Probabilistic Rule Selection matrices
        self.probability_k = 0.05
        self.rule_probabilities = numpy.ones(self.num_lowtier_rules + self.num_hightier_rules)
        self.rule_probabilities_normal = numpy.ones(self.num_lowtier_rules + self.num_hightier_rules)
        self.normalize_rule_probabilities()

        # Initialize iteration count for individual agent
        self.iteration_count = 1

        self.simulation_data = []

    def iterate(self):
        # Deep copy the candidate solution
        self.candidate_solution = copy.deepcopy(self.current_solution)

        # Decide which action to take
        self.apply_lowtier_rule = False
        self.apply_hightier_rule = False

        # Select rule based on specified method
        # self.markov_rule_select()
        # self.probabilistic_rule_select()
        self.random_rule_select()
        # self.previous_tier_preferred_rule_select()

        # Depending which tier the agent prefers in this iteration, implement one rule from that tier
        if self.apply_lowtier_rule:
            print('LT:', self.rule)
            self.candidate_solution.lowtier_rule_perform(self.rule)
        elif self.apply_hightier_rule:
            print('HT:', self.rule)
            self.candidate_solution.hightier_rule_perform(self.rule)

        # Deep copy the candidate to evaluate. Copying allows the candidate's position to be 'reset' after evaluation
        self.candidate_solution_for_evaluation = copy.deepcopy(self.candidate_solution)

        # Apply that action and evaluate the outcome
        self.previous_solution_quality = self.current_solution_quality
        self.rule_accepted = 0
        self.error = 'none'
        if self.candidate_solution.is_valid():
            # self.candidate_solution_for_evaluation.display_visual = True
            self.candidate_results = self.candidate_solution_for_evaluation.evaluate()
            if self.candidate_solution_for_evaluation.stable_system:
                # Decide whether to keep the current solution or revert.
                # Compare quality of candidate solution to current solution
                self.evaluate()
                self.past_candidates_results.append(self.candidate_solution_quality)
            else:
                self.error = 'unstable'
        else:
            self.candidate_solution_quality = self.previous_solution_quality
            self.error = 'invalid'

        # ['repetition', 'iteration', 'rule tier', 'rule number', 'quality before rule', 'quality after rule',
        # 'current solution quality', 'rule acceptance', 'lower tier preference', 'higher tier preference',
        # 'error', 'markov', **'results']
        self.simulation_data.append([0, self.iteration_count, self.preferred_rule_tier, self.rule,
                                     self.previous_solution_quality, self.candidate_solution_quality,
                                     self.current_solution_quality, self.rule_accepted,
                                     self.tier_weights[0], self.tier_weights[1], self.error,
                                     numpy.copy(self.rule_probabilities), self.current_results[0],
                                     min(self.current_results[1]), self.current_results[2]])

        self.iteration_count += 1
        self.tier_weights = (self.tier_weights[0] + self.tier_weight_change[0],
                             self.tier_weights[1] + self.tier_weight_change[1])

        # Tune approach based on evaluation and decision.

    def evaluate(self):
        # Calculate solution quality of current solution and candidate solution
        # self.current_solution_quality = sum([self.current_results[i] * self.weights[i] *
        # self.evaluation_adjustments[i] for i in range(heuristic_bursts.solution.Solution.number_of_metrics)])
        # self.candidate_solution_quality = sum([self.candidate_results[i] * self.weights[i] *
        # self.evaluation_adjustments[i] for i in range(heuristic_bursts.solution.Solution.number_of_metrics)])

        # Used for WEC problem
        # self.current_solution_quality = self.current_results[1]/self.current_results[0]
        # self.candidate_solution_quality = self.candidate_results[1]/self.candidate_results[0]

        # Used for TRUSS problem
        self.current_solution_quality = -self.current_results[0] - pow(100*max(0, self.current_results[2] - min(self.current_results[1])), 2)
        self.candidate_solution_quality = -self.candidate_results[0] - pow(100*max(0, self.candidate_results[2] - min(self.candidate_results[1])), 2)

        print("current quality:", self.current_solution_quality)
        print("candidate quality:", self.candidate_solution_quality)
        print('')

        # Change in quality
        d_quality = self.candidate_solution_quality - self.current_solution_quality
        # d_quality = 1

        # If quality shows candidate is better, accept candidate
        if d_quality > 0:
            self.current_solution = copy.deepcopy(self.candidate_solution)
            self.current_results = self.candidate_results
            self.current_solution_quality = self.candidate_solution_quality
            self.rule_accepted = 1

        # If quality shows candidate is worse, probabilistically accept
        else:
            probability = numpy.exp((self.candidate_solution_quality - self.current_solution_quality)/self.temperature)
            random_num = random.random()
            if random_num <= probability:
                self.current_solution = copy.deepcopy(self.candidate_solution)
                self.current_results = self.candidate_results
                self.current_solution_quality = self.candidate_solution_quality
                self.rule_accepted = 1

            print('probability:', probability)
            print('temp:', self.temperature)
            print('')

        self.all_solution_qualities.append(self.current_solution_quality)

        # Update Markov matrix
        self.update_markov(d_quality)

        # Update last rule and rule tier
        self.last_rule = self.rule
        self.last_tier = self.preferred_rule_tier

        # Update Probability array
        self.update_rule_probabilities(d_quality)

        # Update temperature every 10th iteration
        if self.iteration_count % 10 == 0:
            self.update_temp()

    def update_temp(self):
        self.candidate_variance = numpy.var(self.past_candidates_results)
        self.temperature = self.temperature*(1-(self.temperature*self.TrikiParameter)/self.candidate_variance)
        if self.temperature < 1.0 * pow(10, -10):
            self.temperature = 1.0 * pow(10, -10)
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
        self.simulation_data = self.team_all_simulation_data[solution_selected]
        # self.current_solution_quality = sum([self.current_results[i] * self.weights[i]
        # * self.evaluation_adjustments[i] for i in range(heuristic_bursts.solution.Solution.number_of_metrics)])

        # For WEC
        # self.current_solution_quality = self.current_results[1]/self.current_results[0]

        # For TRUSS
        self.current_solution_quality = -self.current_results[0] - pow(100*max(0, self.current_results[2] - min(self.current_results[1])), 2)

    def random_rule_select(self):
        # Probabilistically select rule tier based on preferences
        self.preferred_rule_tier = str(numpy.random.choice(self.tiers, p=self.tier_weights))

        if self.preferred_rule_tier == 'low':
            self.apply_lowtier_rule = True
        elif self.preferred_rule_tier == 'high':
            self.apply_hightier_rule = True

        # Depending which tier the agent prefers in this iteration, implement one rule from that tier
        if self.apply_lowtier_rule:
            self.rule = self.candidate_solution.lowtier_rule_select()
        elif self.apply_hightier_rule:
            self.rule = self.candidate_solution.hightier_rule_select()

    def previous_tier_preferred_rule_select(self):
        # Choose next rule tier based on previous rule tier. Selection is weighted to prefer last tier
        if self.last_tier == 'low':
            self.preferred_rule_tier = str(numpy.random.choice(self.tiers, p=self.previous_tier_preferred_selection_weights[0]))
        elif self.last_tier == 'high':
            self.preferred_rule_tier = str(numpy.random.choice(self.tiers, p=self.previous_tier_preferred_selection_weights[1]))

        if self.preferred_rule_tier == 'low':
            self.apply_lowtier_rule = True
        elif self.preferred_rule_tier == 'high':
            self.apply_hightier_rule = True

        # Depending which tier the agent prefers in this iteration, implement one rule from that tier
        if self.apply_lowtier_rule:
            self.rule = self.candidate_solution.lowtier_rule_select()
        elif self.apply_hightier_rule:
            self.rule = self.candidate_solution.hightier_rule_select()

    def markov_rule_select(self):
        # Array of all rule indices
        all_rules = numpy.arange(self.num_lowtier_rules + self.num_hightier_rules)

        # Determine markov index of last rule applied
        if self.last_tier == 'low':
            last_rule_index = self.last_rule - 1
        elif self.last_tier == 'high':
            last_rule_index = self.last_rule + self.num_lowtier_rules - 2

        # Extract probabilities for next rule based on last
        rule_probabilities = self.markov_normal[last_rule_index]

        # Probabilistically select next rule to be applied
        next_rule = numpy.random.choice(all_rules, p=rule_probabilities)
        if next_rule < self.num_lowtier_rules:
            self.rule = next_rule + 1
            self.preferred_rule_tier = "low"
        elif next_rule >= self.num_lowtier_rules:
            self.rule = next_rule - self.num_lowtier_rules + 2
            self.preferred_rule_tier = "high"

        if self.preferred_rule_tier == 'low':
            self.apply_lowtier_rule = True
        elif self.preferred_rule_tier == 'high':
            self.apply_hightier_rule = True

    def probabilistic_rule_select(self):
        # Array of all rule indices
        all_rules = numpy.arange(self.num_lowtier_rules + self.num_hightier_rules)

        # Probabilistically select next rule to be applied
        next_rule = numpy.random.choice(all_rules, p=self.rule_probabilities_normal)
        if next_rule < self.num_lowtier_rules:
            self.rule = next_rule + 1
            self.preferred_rule_tier = "low"
        elif next_rule >= self.num_lowtier_rules:
            self.rule = next_rule - self.num_lowtier_rules + 2
            self.preferred_rule_tier = "high"

        if self.preferred_rule_tier == 'low':
            self.apply_lowtier_rule = True
        elif self.preferred_rule_tier == 'high':
            self.apply_hightier_rule = True

    def update_markov(self, d_quality):
        # Determine markov index of last rule applied
        if self.last_tier == 'low':
            last_rule_index = self.last_rule - 1
        elif self.last_tier == 'high':
            last_rule_index = self.last_rule + self.num_lowtier_rules - 2

        # Determine markov index of rule just applied
        if self.preferred_rule_tier == 'low':
            rule_index = self.rule - 1
        elif self.preferred_rule_tier == 'high':
            rule_index = self.rule + self.num_lowtier_rules - 2

        # Update markov matrix based on performance of rule on solution quality
        if d_quality > 0:
            self.markov_matrix[last_rule_index][rule_index] *= (1 + self.markov_k)
        elif d_quality < 0:
            self.markov_matrix[last_rule_index][rule_index] *= (1 - self.markov_k)
        else:
            pass

        self.normalize_markov()

    def update_rule_probabilities(self, d_quality):
        # Determine markov index of rule just applied
        if self.preferred_rule_tier == 'low':
            rule_index = self.rule - 1
        elif self.preferred_rule_tier == 'high':
            rule_index = self.rule + self.num_lowtier_rules - 2

        # Update markov matrix based on performance of rule on solution quality
        if d_quality > 0:
            self.rule_probabilities[rule_index] *= (1 + self.probability_k)
        elif d_quality < 0:
            self.rule_probabilities[rule_index] *= (1 - self.probability_k)
        else:
            pass

        self.normalize_rule_probabilities()

    def normalize_markov(self):
        for i in range(0, len(self.markov_matrix)):
            self.markov_normal[i] = self.markov_matrix[i] / sum(self.markov_matrix[i])

    def normalize_rule_probabilities(self):
        self.rule_probabilities_normal = self.rule_probabilities / sum(self.rule_probabilities)
