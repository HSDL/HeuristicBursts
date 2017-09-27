from team import Team
import os as os


class Batch(object):

    def __init__(self, solution_class, number_of_agents, number_of_teams):
        self._print_solution_class(solution_class)
        self.team_list = []
        for i in range(number_of_teams):
            self.team_list.append(Team(number_of_agents))

    @staticmethod
    def _print_solution_class(solution_class):
        solution_file = solution_class.lower()

        try:
            os.remove('solution.py')
        except OSError:
            pass

        with open('solution.py', 'w') as fid:
            fid.write('from %s import %s\n\n\n' % (solution_file, solution_class))
            fid.write('class Solution(%s):\n' % solution_class)
            fid.write('    def __init__(self):\n')
            fid.write('        %s.__init__(self)\n' % solution_class)