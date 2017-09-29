from heuristic_bursts.batch import Batch

def test_creation_of_batch():

    a = Batch('WEC', 3, 3)

    print(a.team_list)

    assert(len(a.team_list)==3)
