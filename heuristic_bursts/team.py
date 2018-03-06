import heuristic_bursts.agent
import copy


class Team(object):

    def __init__(self, options):
        self.options = options
        self.agent_list = []
        self.current_solutions = []
        self.current_qualities = []
        for i in range(self.options.number_of_agents):
            self.agent_list.append(heuristic_bursts.agent.Agent(self.options))

    def run(self):
        for i in range(1, self.options.number_of_iterations + 1):
            print("Iteration:", i)
            self.iterate()
            if self.options.is_scheduled() and i % self.options.interaction_period == 0:
                print("")
                print("INTERACTION PERIOD")
                print("")
                self.interact()

    def iterate(self):
        for agent in self.agent_list:
            agent.iterate()

    def interact(self):
        # Pull agents current solutions
        self.current_solutions = []
        self.current_qualities = []
        for agent in self.agent_list:
            self.current_solutions.append(agent.current_solution)
            self.current_qualities.append(agent.current_solution_quality)
            print(agent.current_results)

        print('')
        # Share those solutions
        for agent in self.agent_list:
            agent.team_current_solutions = copy.deepcopy(self.current_solutions)
            agent.team_current_qualities = copy.deepcopy(self.current_qualities)
            agent.interact()
