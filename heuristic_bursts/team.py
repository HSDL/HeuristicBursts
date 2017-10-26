import heuristic_bursts


class Team(object):

    def __init__(self, options):
        self.options = options
        self.agent_list = []
        for i in range(self.options.number_of_agents):
            self.agent_list.append(heuristic_bursts.agent.Agent(self.options))

    def run(self):
        for i in range(self.options.number_of_iterations):
            self.iterate()

    def iterate(self):
        for agent in self.agent_list:
            agent.iterate()
