import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        # test_constraint1 = {'agent': 0,
        #  'loc': [(1, 5)],
        #  'timestep': 4}
        # test_constraint2 = {'agent': 1,
        #  'loc': [(1, 2), (1,3)],
        #  'timestep': 1}
        # test_goal_constraint = {'agent': 0,
        #  'loc': [(1, 5)],
        #  'timestep': 10}
        #
        # constraints.append(test_constraint1)
        # constraints.append(test_constraint2)
        # constraints.append(test_goal_constraint)
        high_priority_nums = self.num_of_agents / 2
        env_size = len(self.my_map) * len(self.my_map[0])
        # time_for_sol = 1
        time_for_sol = env_size * env_size
        sol_timer = timer.time()

        for i in range(self.num_of_agents):  # Find path for each agent
            # time constraints for 2.4: I am adding a constraints for all the places except goals in the map once the
            # timestep is above the environment size^2*(high priority agent len) so that a_star will raise no solutions
            # when the time is due
            row_num = 0
            for row in self.my_map:
                col_num = 0
                for col in row:
                    if (row_num, col_num) != self.goals[i]:
                        constraints.append({'agent': i, 'loc': [(row_num, col_num)], 'timestep': time_for_sol, 'positive': False})
                    col_num += 1
                row_num += 1

            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            else:
                path_len = len(path)
                # filling the upper bound for 2.4
                if i < high_priority_nums:
                    time_for_sol = time_for_sol*path_len
                for j in range(self.num_of_agents):
                    if i < j:
                        k = 0
                        for p in path:
                            constraints.append({'agent': j, 'loc': [p], 'timestep': k, 'positive': False})
                            k += 1
                        for e in range(path_len - 1):
                            edge = [path[e + 1], path[e]]
                            constraints.append({'agent': j, 'loc': edge, 'timestep': e + 1, 'positive': False})
                        # goal constraint for other agents
                        constraints.append({'agent': j, 'loc': [path[-1]], 'timestep': -k, 'positive': False})
            result.append(path)

            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result,self.goals)))
        print(result)
        return result
