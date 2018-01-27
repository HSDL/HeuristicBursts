import numpy
import copy
import heuristic_bursts.solution


class Agent(object):
    # TODO: Double-check iteration comparison
    def __init__(self, options):
        self.options = options
        self.current_solution = heuristic_bursts.solution.Solution()
        self.team_current_solutions = []
        self.metrics = []
        self.weights = [1/3., 1/3., 1/3.]
        self.comparison_inversions = [-1, 1, -1] # This is used in comparing solutions by ratio
        self.current_solution_quality = numpy.Inf
        if options.learning_style is 'multinomial':
            self.highertier_weights = numpy.ones(heuristic_bursts.solution.Solution.number_of_highertier_operations)
        elif options.learning_style is 'markovian':
            self.highertier_weights = numpy.ones([heuristic_bursts.solution.Solution.number_of_highertier_operations,
                                                  heuristic_bursts.solution.Solution.number_of_highertier_operations])
        self.preferred_rule_tier = 'low'
        current_solution_for_evaluation = copy.deepcopy(self.current_solution)
        self.current_results = current_solution_for_evaluation.evaluate()

    def iterate(self):
        # Deep copy the candidate solution
        candidate_solution = copy.deepcopy(self.current_solution)

        # Decide which action to take
        apply_lowtier_rule = False
        apply_hightier_rule = False

        # Each tier has its own weight for selection
        tiers = ['low', 'high']
        tier_weights = [0.0, 1.0]
        self.preferred_rule_tier = str(numpy.random.choice(tiers, p=tier_weights))
        print('Preference:', self.preferred_rule_tier)
        print('')
        if self.preferred_rule_tier == 'low':
            apply_lowtier_rule = True
        elif self.preferred_rule_tier == 'high':
            apply_hightier_rule = True

        # Depending which tier the agent prefers in this iteration, implement one rule from that tier
        if apply_lowtier_rule:
            rule = candidate_solution.lowtier_rule_select()
            candidate_solution.lowtier_rule_perform(rule)
        elif apply_hightier_rule:
            rule = candidate_solution.hightier_rule_select()
            candidate_solution.hightier_rule_perform(rule)
        print("Rule Selected: ", rule)
        # Deep copy the candidate to evaluate. Copying allows the candidate's position to be 'reset' after evalutation
        candidate_solution_for_evaluation = copy.deepcopy(candidate_solution)

        # Apply that action and evaluate the outcome
        if candidate_solution.is_valid:
            candidate_results = candidate_solution_for_evaluation.evaluate()
            if candidate_solution_for_evaluation.stable_system:
                # Decide whether to keep the current solution or revert.
                # Compare quality of candidate solution to current solution
                weighted_comparison = self.compare_solutions(self.current_results, candidate_results)
                print('Current Results:', self.current_results)
                print('Candidate Results:', candidate_results)
                print('Weighted Comparison Ratio:', weighted_comparison)
                print('')
                if weighted_comparison > 1.0:
                    self.current_solution = copy.deepcopy(candidate_solution)
                    self.current_results = candidate_results

        # Tune approach based on evaluation and decision.

    def compare_solutions(self, solution_a_results, solution_b_results):
        weigted_metric_comparison = [pow((solution_b_results[i]/solution_a_results[i]), self.comparison_inversions[i])
                                     *self.weights[i] for i in range(heuristic_bursts.solution.Solution.number_of_metrics)]
        result_ratio = sum(weigted_metric_comparison)
        return result_ratio

    def evaluate(self):
        self.metrics = self.current_solution.evaluate()
        self.current_solution_quality = [self.metrics[i]*self.weights[i] for i in range(heuristic_bursts.solution.Solution.number_of_metrics)]

    def interact(self):
        asdf = 1
        # Look at other solutions and see what you like
