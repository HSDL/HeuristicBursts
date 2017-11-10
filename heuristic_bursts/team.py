import heuristic_bursts.agent


class Team(object):

    def __init__(self, options):
        self.options = options
        self.agent_list = []
        self.current_solutions = []
        for i in range(self.options.number_of_agents):
            self.agent_list.append(heuristic_bursts.agent.Agent(self.options))

    def run(self):
        for i in range(self.options.number_of_iterations):
            self.iterate()
            if self.options.is_scheduled() and i % self.options.interaction_period == 0:
                self.interact()

    def iterate(self):
        for agent in self.agent_list:
            agent.iterate()

    def interact(self):
        # Pull agents current solutions
        current_solutions = []
        for agent in self.agent_list:
            self.current_solutions.append(agent.current_solution)

        # Share those solutions
        for agent in self.agent_list:
            agent.team_current_solutions = current_solutions
            agent.interact()
