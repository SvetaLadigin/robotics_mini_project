import copy
import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

""" detect collision accepts to paths and returns a collision: {'loc': loc,'timestep': i, 'positive': False} """

def detect_collision(path1, path2):
    len_path1 = len(path1)
    len_path2 = len(path2)
    min_len = 0
    max_len = 0
    min_path = path1
    max_path = path2
    if len_path1 <= len_path2:
        min_len = len_path1
        max_len = len_path2
    else:
        min_len = len_path2
        max_len = len_path1
        min_path = path2
        max_path = path1
    # vertex collision:
    for i in range(min_len):
        if get_location(path1,i) == get_location(path2,i):
            # print("detect 1 coll")
            # print({'loc': [get_location(path1, i)],'timestep': i})
            return {'loc': [get_location(path1, i)],'timestep': i,'goal':0, 'positive': False}
    # edge collision:
    for i in range(min_len-1):
        if get_location(path1,i) == get_location(path2,i+1) and get_location(path1, i+1) == get_location(path2, i):
            return {'loc': [get_location(path1, i), get_location(path1, i+1)],'goal':0, 'timestep': i+1, 'positive': False}
    # goal collision
    # without this constraint results are the same as theirs but this constraint make sure
    # that after the agent arrived in its goal location no other agent would cross his cell
    goal_location = get_location(min_path,min_len-1)
    for i in range(max_len-min_len):
        if get_location(max_path,i+min_len-1) == goal_location:
            return {'loc': [goal_location], 'timestep': i+min_len-1,'goal':1, 'positive': False}
    return None


    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.


"""detect collisions : gets a list of paths and returns a combined collision : 
{'a1': i, 'a2': j, 'loc': loc, 'timestep': i , 'positive': False} """

def detect_collisions(paths):
    ##############################
    collisions = []
    # print("all paths")
    # print(paths)
    i = 0
    for path1 in paths:
        j = 0
        for path2 in paths:
            if i != j:
                collision = detect_collision(path1, path2)
                if collision:
                    # if collision['goal'] == 1:
                    #     if collision['timestep']> len(path1)-1:
                    #         collision_to_add = {'a1': i, 'a2': j, 'loc': collision['loc'],
                    #                             'timestep': collision['timestep'],'goal':collision['goal'], 'positive': False}
                    #     elif collision['timestep']>len(path2)-1:
                    #         collision_to_add = {'a1': j, 'a2': i, 'loc': collision['loc'],
                    #                             'timestep': collision['timestep'],'goal':collision['goal'], 'positive': False}
                    # else:
                    collision_to_add = {'a1': i, 'a2': j, 'loc': collision['loc'], 'timestep': collision['timestep'],'goal':collision['goal'], 'positive': False}
                    collisions.append(collision_to_add)
            j += 1
        i += 1
    # print(collisions)
    return collisions
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.


""" splits the collision into 2 constraints"""

def standard_splitting(collision):
    seperated_collisions = []
    loc = collision['loc']
    # if collision['goal'] == 1:
    #     collision1 = {'agent': collision['a1'], 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
    #     collision2 = {'agent': collision['a2'], 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
    #     seperated_collisions.append(collision1)
    #     seperated_collisions.append(collision2)
    #
    # else:
    if len(loc) == 1:
        collision1 = {'agent': collision['a1'], 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
        collision2 = {'agent': collision['a2'], 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
    else:
        collision1 = {'agent': collision['a1'], 'loc': [loc[0],loc[1]], 'timestep': collision['timestep'], 'positive': False}
        collision2 = {'agent': collision['a2'], 'loc': [loc[1],loc[0]], 'timestep': collision['timestep'], 'positive': False}
    seperated_collisions.append(collision1)
    seperated_collisions.append(collision2)
    #print(seperated_collisions)

    return seperated_collisions
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep


""" splits the collision into 2 constraints of negative and one positive chosen randomly"""

def disjoint_splitting(collision):
    ##############################
    disjoint_constraints = []
    positive_agent = random.randint(0,1)
    if positive_agent == 0:
        agent1 = collision['a1']
        agent2 = collision['a2']
    else:
        agent2 = collision['a1']
        agent1 = collision['a2']
    loc = collision['loc']
    if len(loc) == 1 and collision['goal'] == 0:
        collision1 = {'agent': agent1, 'loc': loc, 'timestep': collision['timestep'], 'positive': True}
        collision2 = {'agent': agent2, 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
    else:
        collision1 = {'agent': agent1, 'loc': [loc[0], loc[1]], 'timestep': collision['timestep'], 'positive': True}
        collision2 = {'agent': agent2, 'loc': [loc[1], loc[0]], 'timestep': collision['timestep'], 'positive': False}
    disjoint_constraints.append(collision1)
    disjoint_constraints.append(collision2)
    print(disjoint_constraints)
    return disjoint_constraints

    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly



class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def paths_violate_constraint(self, constraint, paths):
        ids = []
        loc = constraint['loc']
        len_of_loc = len(loc)
        # print(len(loc))
        if len_of_loc == 1:
            i = 0
            for path in paths:
                if i != constraint['agent']:
                    for j in range(len(path)):
                        if j == constraint['timestep'] and get_location(path, j) == loc[0]:
                            ids.append(i)
                i +=1
        if len_of_loc == 2:
            i = 0
            for path in paths:
                for j in range(len(path)-1):
                    if constraint['timestep'] == j + 1:
                        current_loc = get_location(path,j)
                        next_loc = get_location(path, j+1)
                        if current_loc == loc[1] and next_loc == loc[0]:
                            ids.append(i)
                        if current_loc == loc[0] and next_loc == loc[1]:
                            ids.append(i)
                i +=1
        print(ids)
        return ids



    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'],self.goals,self.starts)
        #print(root['cost'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
        while self.open_list:
            p = self.pop_node()
            if len(p['collisions']) == 0:
                self.print_results(root)
                return p['paths']
            collision = p['collisions'][0]
            # choosing splitting method depending on disjoint
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)
            violate = 0
            for constraint in constraints:
                # Task 4 #########################################
                # if disjoint was on and we have a positive constraint we will be
                # adding negative constraints for every agent that is not constraint['agent']
                if constraint['positive']:
                    new_constraints = []
                    loc = constraint['loc']
                    for a in range(self.num_of_agents):
                        # print("constraint agent")
                        # print(constraint['agent'])
                        if a != constraint['agent']:
                            if len(loc) == 2:
                                new_constraints.append({'agent': a, 'loc': [loc[1], loc[0]], 'timestep': collision['timestep'], 'positive': False})
                            elif len(loc) == 1:
                                new_constraints.append(
                                    {'agent': a, 'loc': loc, 'timestep': collision['timestep'],
                                     'positive': False})
                        print("new const")
                        print(new_constraints)
                    # for c in new_constraints:
                    #     p['constraints'].append(c)
                    # if violate > 0:
                    #     ids = self.paths_violate_constraint(constraint,p['paths'])
                    #     for i in ids:
                    #         path_i = a_star(self.my_map,self.starts[i],self.goals[i],self.heuristics[i],i,p['constraints'])
                    # print(violate)
                # if we have more then 1 agent who doesnt have a path we wont be adding the node
                ##############################################

                q = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
                const_for_q = []
                for constraint1 in p['constraints']:
                    const_for_q.append(constraint1)
                # const_for_q = copy.deepcopy(p['constraints'])
                if constraint not in p['constraints']:
                    if not constraint['positive']:
                        const_for_q.append(constraint)
                # if disjoint is on we have new constraints
                if disjoint:
                    print("adding new")
                    if new_constraints:
                        for c in new_constraints:
                            if constraint not in const_for_q:
                                const_for_q.append(c)
                    print(const_for_q)
                q['constraints'] = const_for_q
                q['paths'] = p['paths']
                if disjoint:
                    ids = self.paths_violate_constraint(constraint, q['paths'])
                    for i in ids:
                        path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i,
                                      q['constraints'])
                        if path:
                            if len(path) > 0:
                                replaced_q_paths = q['paths'][:i] + [path] + q['paths'][i + 1:]
                                q['paths'] = copy.deepcopy(replaced_q_paths)
                                new_collisions = detect_collisions(q['paths'])
                                q['collisions'] = new_collisions
                                q['cost'] = get_sum_of_cost(q['paths'], self.goals, self.starts)
                else:
                    a_i = constraint['agent']
                    path = a_star(self.my_map,self.starts[a_i],self.goals[a_i],self.heuristics[a_i],a_i,q['constraints'])
                    if path:
                        if len(path)>0:
                            replaced_q_paths = q['paths'][:a_i]+[path]+q['paths'][a_i+1:]
                            q['paths'] = copy.deepcopy(replaced_q_paths)
                            #q['paths'] = replaced_q_paths
                            new_collisions = detect_collisions(replaced_q_paths)
                            q['collisions'] = new_collisions
                            q['cost'] = get_sum_of_cost(replaced_q_paths,self.goals,self.starts)
                if disjoint:
                    violate = len(self.paths_violate_constraint(constraint, q['paths']))
                    if violate == 0:
                        self.push_node(q)
                else:
                    self.push_node(q)
        # raise BaseException('No solutions')
        print("open list is empty")

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        # print(get_sum_of_cost(root['paths']))
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'], self.goals,self.starts)))
        print(node['paths'])
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
