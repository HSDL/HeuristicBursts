import heuristic_bursts.agent
import copy

# TODO: DELETE THIS
import wec.wec_visual


class Team(object):

    def __init__(self, options):
        self.options = options
        self.agent_list = []
        self.current_solutions = []
        self.current_results = []
        self.current_qualities = []
        self.all_qualities = []
        self.all_simulation_data = []

        for i in range(self.options.number_of_agents):
            self.agent_list.append(heuristic_bursts.agent.Agent(self.options))

        self.display = wec.wec_visual.wec_visual()

    def run(self):
        for i in range(1, self.options.number_of_iterations + 1):
            print("Iteration:", i)
            print("")
            self.iterate()
            # print(self.agent_list[0].markov_matrix)
            if self.options.is_scheduled() and i % self.options.interaction_period == 0:
                print("")
                print("INTERACTION PERIOD")
                print("")
                self.interact()
            self.display.display(self.agent_list[0].current_solution)

    def iterate(self):
        for agent in self.agent_list:
            agent.iterate()

    def interact(self):
        # Pull agents current solutions
        self.current_solutions = []
        self.current_results = []
        self.current_qualities = []
        self.all_qualities = []
        self.all_simulation_data = []
        for agent in self.agent_list:
            self.current_solutions.append(agent.current_solution)
            self.current_results.append(agent.current_results)
            self.current_qualities.append(agent.current_solution_quality)
            self.all_qualities.append(agent.all_solution_qualities)
            self.all_simulation_data.append(agent.simulation_data)
            print(agent.current_results)
        print('')

        # Share those solutions
        for agent in self.agent_list:
            agent.team_current_solutions = copy.deepcopy(self.current_solutions)
            agent.team_current_results = copy.deepcopy(self.current_results)
            agent.team_current_qualities = copy.deepcopy(self.current_qualities)
            agent.team_all_qualities = copy.deepcopy(self.all_qualities)
            agent.team_all_simulation_data = copy.deepcopy(self.all_simulation_data)
            agent.interact()
