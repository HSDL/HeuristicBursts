class Options(object):

    solution_class = ''
    number_of_agents = 1
    number_of_teams = 1
    number_of_iterations = 100
    learning_style = 'multinomial'
    interaction_period = 10
    interaction_style = 'scheduled'

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

        # Check to make sure the number of agents is positive non-zero
        if self.number_of_agents <= 0:
            raise ValueError('number_of_agents must be greater than 0')

        # Check to make sure the number of teams is positive non-zero
        if self.number_of_teams <= 0:
            raise ValueError('number_of_teams must be greater than 0')

        # Check to make sure the number of iterations is positive non-zero
        if self.number_of_iterations <= 0:
            raise ValueError('number_of_iterations must be greater than 0')

        # Check to make sure the number of iterations is greater than 1
        if self.interaction_period < 1:
            raise ValueError('interaction_period must not be less than 1')

        # Check to make sure learning_style is a valid option
        learning_options = ['none', 'multinomial', 'markovian']
        if self.learning_style not in learning_options:
            raise ValueError('learning_style must be one of '+", ".join(learning_options))

        # Check to make sure interaction_style is a valid option
        interaction_options = ['scheduled', 'random']
        if self.interaction_style not in interaction_options:
            raise ValueError('interaction_style must be one of '+", ".join(interaction_options))


