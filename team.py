from agent import Agent


class Team(object):

    def __init__(self, number_of_agents):
        self.agent_list = []
        for i in range(number_of_agents):
            self.agent_list.append(Agent())

    def iterate(self):
        for agent in self.agent_list:
            agent.iterate()