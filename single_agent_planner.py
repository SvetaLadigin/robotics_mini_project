import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths, goals,starts):
    rst = 0
    i = 0
    for path in paths:
        print(path)
        t = 1
        first_goal_index = 0
        j = 0
        for p in path:
            if p == goals[i]:
                first_goal_index = j
                break
            j +=1

        for loc in path:
            # print(loc)
            # print(goals[i])
            if loc != goals[i] and t <= first_goal_index:
                rst +=1
            t +=1
        i +=1
        # rst += len(path) - 1
    return rst
    # rst = 0
    # for path in paths:
    #     rst += len(path) - 1
    # print(rst)
    # return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


"""build_constraint_table: gets constraints list and agent and returns a dict with constraints for the given agent 
with time steps as keys """

def build_constraint_table(constraints, agent):
    constraints_table_for_agent = {}
    list_constraints_for_agent = []
    for constraint in constraints:
        if constraint['agent'] == agent:
            list_constraints_for_agent.append(constraint)
    for constraint in list_constraints_for_agent:
        if constraint['timestep'] in constraints_table_for_agent:
            # print(constraint)
            new_list_constraints = constraints_table_for_agent[constraint['timestep']]
            new_list_constraints.append(constraint)
            constraints_table_for_agent[constraint['timestep']] = new_list_constraints
            # print(constraints_table_for_agent[constraint['timestep']])
        else:
            constraints_table_for_agent[constraint['timestep']] = [constraint]

    # print(constraints_table_for_agent)
    return constraints_table_for_agent

    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


""" checks if the loc is within map boundries """

def not_within_map(my_map, loc):
    if loc[0] < 0 or loc[1] < 0:
        return True
    num_of_rows = len(my_map)
    num_of_col = len(my_map[0])
    if loc[0] > num_of_rows - 1 or loc[1] > num_of_col - 1:
        return True
    return False


""" checks if the constraint is a positive one or negative one"""

def is_constrained_positive(curr_loc, next_loc, next_time, constraint_table):
    if next_time in constraint_table:
        for constraint in constraint_table[next_time]:
            if len(constraint['loc']) == 1:
                if next_loc == constraint['loc'][0]:
                    if constraint['positive']:
                        return True

            if len(constraint['loc']) == 2:
                if curr_loc == constraint['loc'][0] and next_loc == constraint['loc'][1]:
                    if constraint['positive']:
                        return True
    return False


""" checks if there is a negative constraint on next location """

def is_constrained_negative(curr_loc, next_loc, next_time, constraint_table):
    minus_next_time = -next_time
    if next_time in constraint_table:
        for constraint in constraint_table[next_time]:
            if len(constraint['loc']) == 1:
                if next_loc == constraint['loc'][0]:
                    if not constraint['positive']:
                        return True

            if len(constraint['loc']) == 2:
                if curr_loc == constraint['loc'][0] and next_loc == constraint['loc'][1]:
                    if not constraint['positive']:
                        return True

    # if other agent in its goal
    for key in constraint_table:
        if 0 > key >= minus_next_time:
            for constraint in constraint_table[key]:
                if len(constraint['loc']) == 1:
                    if constraint['loc'][0] == next_loc:
                        if not constraint['positive']:
                            return True

    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    return False


"""checks if there is a goal constraint for the agent's goal location """

def goal_constraint(next_loc, goal, next_time, table_of_constraints):
    for key in table_of_constraints:
        if key == next_time:
            for constraint in table_of_constraints[key]:
                if len(constraint['loc']) == 1:
                    if constraint['loc'][0] == goal and next_loc == goal:
                        if not constraint['positive']:
                            return True
    return False


def goal_constraint_later_time(next_loc, goal, next_time, table_of_constraints):
    for key in table_of_constraints:
        if key >= next_time:
            for constraint in table_of_constraints[key]:
                if len(constraint['loc']) == 1:
                    if constraint['loc'][0] == goal and next_loc == goal:
                        if not constraint['positive']:
                            return True
                if len(constraint['loc']) == 2:
                    if constraint['loc'][1] == goal or constraint['loc'][0] == goal:
                        if not constraint['positive']:
                            return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraints_for_agent = build_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:
            if not goal_constraint_later_time(curr['loc'],goal_loc,curr['timestep'],constraints_for_agent):
                return get_path(curr)
        # we have 5 direction , left, right, up, down and stay in place
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            # constraint for checking map boundaries
            if not_within_map(my_map, child_loc):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            # checking if we have a vertex or edge or future goal constraint
            if is_constrained_negative(curr['loc'], child_loc, curr['timestep'] + 1,
                                       constraints_for_agent):
                continue
            if goal_constraint(child_loc,goal_loc,curr['timestep']+1,constraints_for_agent):
                continue


            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc']), child['timestep']] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
