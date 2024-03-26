import sys
import numpy
from problem import RobotNavigationProblem
from search_methods import *


def select_search_method(method_name: str, problem: 'RobotNavigationProblem') -> 'SearchMethod':
    method_name = method_name.lower()

    if method_name == 'dfs':
        return DepthFirstSearch(problem)
    elif method_name == 'bfs':
        return BreadthFirstSearch(problem)
    elif method_name == 'gbfs':
        return GreedyBestFirstSearch(problem)
    elif method_name == 'as':
        return AStarSearch(problem)
    elif method_name == 'cus1':
        return IterativeDeepeningSearch(problem)
    elif method_name == 'cus2':
        return BidirectionalAStarSearch(problem)
    elif method_name == 'limited':
        return DepthLimitedSearch(problem, 100)

    raise RuntimeError(f"There is no search method named {method_name.upper()}")


def main():
    if len(sys.argv) < 3:
        raise RuntimeError("Command line arguments are missing")

    filename = sys.argv[1]
    search_method_name = sys.argv[2]

    problem = RobotNavigationProblem(filename)
    search_method = select_search_method(search_method_name, problem)

    search_method.solve()

    print(filename, search_method.method_name)

    if search_method.is_solved():
        print(search_method)
    else:
        print('No goal is reachable;', search_method.nodes_created)

if __name__ == '__main__':
    # problem = RobotNavigationProblem("../data/file.txt")
    # search_method = IterativeDeepeningSearch(problem)
    # search_method.solve()
    # print(search_method, '\n')
    #
    # method_2 = UniformCostSearch(problem)
    # method_2.solve()
    # print(method_2, '\n')
    #
    # method_3 = IterativeDeepeningSearchRecursive(problem)
    # method_3.solve()
    # print(method_3)
    main()
