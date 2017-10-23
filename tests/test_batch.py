import heuristic_bursts.batch


def test_creation_of_batch():

    a = heuristic_bursts.batch.Batch('WEC', 3, 3)

    print(a.team_list)

<<<<<<< HEAD
    assert(len(a.team_list)==3)

#comment
=======
    assert(len(a.team_list) == 3)
>>>>>>> f5763d7252a8846e91881171ae59bbe5b1573b15
