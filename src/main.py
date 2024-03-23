import numpy
from problem import RobotNavigationProblem
from search import *
from search_methods import *


def main():
    problem = RobotNavigationProblem("../data/file.txt")
    result = uniform_cost_search(problem)
    method = BidirectionalAStarSearch(problem)
    method.solve()

    # print('func', result)
    # print(len(result))
    print('method')
    print(method.solution)
    print(len(method.solution))

    if result is None:
        print("No goal is reachable")
    elif len(result) == 2:
        print(result[0], result[1])
        print(result[0].solution())
        print(method.goal_node, method.nodes_created)
       # print(method.solution())
    elif len(result) == 3:
        print(result[0], result[1])
        print(result[2])


if __name__ == '__main__':
    main()
