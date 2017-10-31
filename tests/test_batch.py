import heuristic_bursts.options
import heuristic_bursts.batch


def test_creation_of_batch():

    options = heuristic_bursts.options.Options(solution_class='WEC',
                                               number_of_agents=3,
                                               number_of_teams=3,
                                               learning_style='none')
    a = heuristic_bursts.batch.Batch(options)
    print(a.team_list)
    assert(len(a.team_list) == 3)
