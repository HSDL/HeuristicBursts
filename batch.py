from agent import Agent
import os as os


class Batch(object):

    def __init__(self, solution_class):
        self._print_solution_class(solution_class)

    def run_agents(self, n):
        for i in range(n):
            temp = Agent(i)

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