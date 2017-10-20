import heuristic_bursts.batch


def test_creation_of_batch():

    a = heuristic_bursts.batch.Batch('WEC', 3, 3)

    print(a.team_list)

    assert(len(a.team_list)==3)

#comment