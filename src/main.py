from robot_navigation_problem import RobotNavigationProblem
from uninformed_search import *


def main():
    problem = RobotNavigationProblem("../data/file.txt")
    result = depth_first_search(problem)

    if result is None:
        print("No goal is reachable")
    else:
        print(result[0], result[1])
        print(result[0].solution())


if __name__ == '__main__':
    main()
