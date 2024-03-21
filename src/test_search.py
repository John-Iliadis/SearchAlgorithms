from robot_navigation_problem import RobotNavigationProblem
from search import *


def test_order_of_expansion():
    problem = RobotNavigationProblem('../data/order_1.txt')
    node, count = greedy_best_first_search(problem)

    assert node.state == Vector2i(2, 0)

    problem = RobotNavigationProblem('../data/order_2.txt')
    node, count = greedy_best_first_search(problem)

    assert node.state == Vector2i(0, 2)

    problem = RobotNavigationProblem('../data/order_3.txt')
    node, count = greedy_best_first_search(problem)

    assert node.state == Vector2i(2, 4)
