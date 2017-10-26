class Options(object):

    solution_class = ''
    number_of_agents = 1
    number_of_teams = 1
    number_of_iterations = 100

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)
