import heuristic_bursts.agent
import heuristic_bursts.team
import heuristic_bursts.options

options = heuristic_bursts.options.Options()
team = heuristic_bursts.team.Team(options)
team.run()

sol0 = team.agent_list[0].current_results
sol1 = team.agent_list[1].current_results
sol2 = team.agent_list[2].current_results

print(sol0)
print(sol1)
print(sol2)
