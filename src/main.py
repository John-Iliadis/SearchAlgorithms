from robot_navigation_problem import RobotNavigationProblem
from search import *


def main():
    problem = RobotNavigationProblem("../data/file.txt")
    result = breadth_first_search(problem)
    result = best_first_search(problem, lambda node: node.path_cost)
    result = greedy_best_first_search(problem)
    result = a_star_search(problem)
    result = uniform_cost_search(problem)
    result = depth_first_search(problem)
    result = iterative_deepening_search(problem)

    if result is None:
        print("No goal is reachable")
    elif len(result) == 2:
        print(result[0], result[1])
        print(result[0].solution())
    elif len(result) == 3:
        print(result[0], result[1])
        print(result[2])


if __name__ == '__main__':
    main()
