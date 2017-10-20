import os as os


class Batch(object):

    def __init__(self, solution_class, number_of_agents, number_of_teams):
        self._print_solution_class(solution_class)
        self.team_list = []

        # This line has to go here to allow
        from heuristic_bursts.team import Team

        # Instantiating teams
        for i in range(number_of_teams):
            self.team_list.append(Team(number_of_agents))

    @staticmethod
    def _print_solution_class(solution_class):
        solution_file = solution_class.lower()

        try:
            os.remove('./heuristic_bursts/solution.py')
        except OSError:
            pass

        with open('./heuristic_bursts/solution.py', 'w') as fid:
            fid.write('from %s.%s import %s\n\n\n' % (solution_file, solution_file, solution_class))
            fid.write('class Solution(%s):\n' % solution_class)
            fid.write('    def __init__(self):\n')
            fid.write('        %s.__init__(self)\n' % solution_class)