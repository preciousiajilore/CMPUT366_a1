import time
from search.algorithms import State, dijkstras, a_star
from search.map import Map
import getopt
import sys

def verify_path(start, goal, path, map):
    if path is None:
        return True

    if not (start == path[0]) or not (goal == path[-1]):
        return False

    for i in range(len(path) - 1):
        current = path[i]
        children = map.successors(current)
        
        contains_next = False
        for child in children:
            if child == path[i + 1]:
                contains_next = True
                break

        if not contains_next:
            return False
    return True
        

def main():
    """
    Function for testing your A* and Dijkstra's implementation. There is no need to edit this file.
    Run it with a -help option to see the options available. 
    """
    test_instances = "test-instances/testinstances.txt"
                              
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []  
    nodes_expanded_astar = []
    nodes_expanded_wastar = []

    time_dijkstra = []  
    time_astar = []
    time_wastar = []

    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
    
    solved_all_problems = True

    for i in range(0, len(start_states)):    
        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        path, cost, expanded_diskstra = dijkstras(start, goal, gridded_map)
        time_end = time.time()
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_dijkstra.append(time_end - time_start)
        verified_path = verify_path(start, goal, path, gridded_map)

        if cost != solution_costs[i] or not verified_path:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print("Is the path correct?", verified_path)
            print()

            solved_all_problems = False
        
        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        path, cost, expanded_astar = a_star(start, goal, gridded_map)
        time_end = time.time()

        nodes_expanded_astar.append(expanded_astar)
        time_astar.append(time_end - time_start)

        verified_path = verify_path(start, goal, path, gridded_map)
        if cost != solution_costs[i] or not verified_path:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print("Is the path correct?", verified_path)
            print()

            solved_all_problems = False
        """
        start = start_states[i]
        goal = goal_states[i]

        time_start = time.time()
        path, cost, expanded_wastar = a_star(start, goal, gridded_map, w=1.5)
        time_end = time.time()
        nodes_expanded_wastar.append(expanded_wastar)
        time_wastar.append(time_end - time_start)
        """
    if solved_all_problems: print('The implementation successfully passed all test cases.')

    from search.plot_results import PlotResults
    plotter = PlotResults()
    plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
    plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")
    #plotter.plot_results(nodes_expanded_astar, nodes_expanded_wastar, "Nodes Expanded (A*)", "Nodes Expanded (WA*, w=1.25)","nodes_expanded_wastar")
    #plotter.plot_results(time_astar, time_wastar,"Running Time (A*)", "Running Time (WA*, w=1.25)","running_time_wastar")



if __name__ == "__main__":
    main()