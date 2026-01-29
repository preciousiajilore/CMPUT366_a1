import time
from search.algorithms import State, a_star
from search.map import Map

# reuse the same verify function from main.py
def verify_path(start, goal, path, map):
    if path is None:
        return True
    if not (start == path[0]) or not (goal == path[-1]):
        return False
    for i in range(len(path) - 1):
        current = path[i]
        children = map.successors(current)
        if not any(child == path[i + 1] for child in children):
            return False
    return True

def main():
    test_instances = "test-instances/testinstances.txt"
    gridded_map = Map("dao-map/brc000d.map")

    solved_all = True
    total_expanded = 0
    total_time = 0.0

    with open(test_instances, "r") as f:
        for idx, line in enumerate(f):
            sx, sy, gx, gy, expected = line.strip().split(",")
            start = State(int(sx), int(sy))
            goal = State(int(gx), int(gy))
            expected = float(expected)

            t0 = time.time()
            path, cost, expanded = a_star(start, goal, gridded_map)
            t1 = time.time()

            ok_path = verify_path(start, goal, path, gridded_map)

            if cost != expected or not ok_path:
                solved_all = False
                print(f"Mismatch on instance #{idx}")
                print("Start:", start, "Goal:", goal)
                print("Your cost:", cost, "Expected:", expected)
                print("Path valid?", ok_path)
                print("Expanded:", expanded, "Time:", t1 - t0)
                print()
                # stop early so you can debug the first failure
                break

            total_expanded += expanded
            total_time += (t1 - t0)

    if solved_all:
        print("A_star passed all test instances!")
        print("Total expanded:", total_expanded)
        print("Total time:", total_time)

if __name__ == "__main__":
    main()
