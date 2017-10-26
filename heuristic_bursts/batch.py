import os
import pkg_resources


class Batch(object):

    def __init__(self, options):
        self._print_solution_class(options.solution_class)
        self.team_list = []
        self.options = options

        # This line has to go here to allow
        from heuristic_bursts.team import Team

        # Instantiating teams
        for i in range(self.options.number_of_teams):
            self.team_list.append(Team(self.options))

    def run(self):
        for team in self.team_list:
            team.run()

    @staticmethod
    def _print_solution_class(solution_class):
        solution_file = solution_class.lower()

        try:
            os.remove(pkg_resources.resource_filename('heuristic_bursts', 'solution.py'))
        except OSError:
            pass

        with open(pkg_resources.resource_filename('heuristic_bursts', 'solution.py'), 'w') as fid:
            fid.write('from %s.%s import %s\n\n\n' % (solution_file, solution_file, solution_class))
            fid.write('class Solution(%s):\n' % solution_class)
            fid.write('    def __init__(self):\n')
            fid.write('        %s.__init__(self)\n' % solution_class)