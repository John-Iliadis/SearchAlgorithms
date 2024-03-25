from problem import RobotNavigationProblem
from vector import Vector2i
import search_methods as sm


def test_order_of_expansion():
    problem = RobotNavigationProblem('../data/order_1.txt')
    search_method = sm.GreedyBestFirstSearch(problem)
    search_method.solve()

    assert search_method.goal_node.state == Vector2i(2, 0)

    problem = RobotNavigationProblem('../data/order_2.txt')
    search_method = sm.GreedyBestFirstSearch(problem)
    search_method.solve()

    assert search_method.goal_node.state == Vector2i(0, 2)

    problem = RobotNavigationProblem('../data/order_3.txt')
    search_method = sm.GreedyBestFirstSearch(problem)
    search_method.solve()

    assert search_method.goal_node.state == Vector2i(2, 4)
