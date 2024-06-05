import copy
import search_methods as sm
from problem import RobotNavigationProblem


def test_initial_node_is_goal():
    problem = RobotNavigationProblem("data/core/test_initial_node_is_goal.txt")

    def search_method_test(method_type):
        search_method = method_type(problem)
        search_method.solve()
        assert search_method.is_solved()
        assert search_method.get_solution() == []

    # breadth first search
    search_method_test(sm.BreadthFirstSearch)

    # depth first search
    search_method_test(sm.DepthFirstSearch)

    # uniform cost search
    search_method_test(sm.UniformCostSearch)

    # iterative deepening search
    search_method_test(sm.IterativeDeepeningSearch)

    # greedy best first search
    search_method_test(sm.GreedyBestFirstSearch)

    # A*
    search_method_test(sm.AStarSearch)

    # bidirectional A*
    search_method_test(sm.BidirectionalAStarSearch)


def test_reachable_goal():
    problem = RobotNavigationProblem("data/core/file.txt")

    def search_method_test(method_type):
        search_method = method_type(problem)
        search_method.solve()
        assert search_method.is_solved()

    # breadth first search
    search_method_test(sm.BreadthFirstSearch)

    # depth first search
    search_method_test(sm.DepthFirstSearch)

    # uniform cost search
    search_method_test(sm.UniformCostSearch)

    # iterative deepening search
    search_method_test(sm.IterativeDeepeningSearch)

    # greedy best first search
    search_method_test(sm.GreedyBestFirstSearch)

    # A*
    search_method_test(sm.AStarSearch)

    # bidirectional A*
    search_method_test(sm.BidirectionalAStarSearch)


def test_no_goal_is_reachable():
    problem = RobotNavigationProblem("data/core/test_no_goal_is_reachable.txt")

    def search_method_test(method_type):
        search_method = method_type(problem)
        search_method.solve()
        assert not search_method.is_solved()

    # breadth first search
    search_method_test(sm.BreadthFirstSearch)

    # depth first search
    search_method_test(sm.DepthFirstSearch)

    # uniform cost search
    search_method_test(sm.UniformCostSearch)

    # iterative deepening search
    search_method_test(sm.IterativeDeepeningSearch)

    # greedy best first search
    search_method_test(sm.GreedyBestFirstSearch)

    # A*
    search_method_test(sm.AStarSearch)

    # bidirectional A*
    search_method_test(sm.BidirectionalAStarSearch)


def test_optimal_solution():
    problem = RobotNavigationProblem("data/core/test_optimal_solution.txt")

    def search_method_test(method_type):
        search_method = method_type(problem)
        search_method.solve()
        assert len(search_method.get_solution()) == 14

    # breadth first search
    search_method_test(sm.BreadthFirstSearch)

    # uniform cost search
    search_method_test(sm.UniformCostSearch)

    # iterative deepening search
    search_method_test(sm.IterativeDeepeningSearch)

    # A*
    search_method_test(sm.AStarSearch)

    # bidirectional A*
    search_method_test(sm.BidirectionalAStarSearch)


def test_optimal_solution_multiple_goals():
    problem = RobotNavigationProblem("data/core/test_optimal_solution_multiple_goals.txt")

    def search_method_test(method_type):
        search_method = method_type(problem)
        search_method.solve()
        assert len(search_method.get_solution()) == 25

    # breadth first search
    search_method_test(sm.BreadthFirstSearch)

    # uniform cost search
    search_method_test(sm.UniformCostSearch)

    # iterative deepening search
    search_method_test(sm.IterativeDeepeningSearch)

    # A*
    search_method_test(sm.AStarSearch)

    # bidirectional A*
    search_method_test(sm.BidirectionalAStarSearch)


def test_metamorphic_scenario():
    problem_forward = RobotNavigationProblem("data/core/test_metamorphic_scenario.txt")
    problem_reverse = copy.deepcopy(problem_forward)
    problem_reverse.initial, problem_reverse.goal = problem_reverse.goal[0], [problem_reverse.initial]

    def search_method_test(search_method_type):
        # get forward solution
        search_method = search_method_type(problem_forward)
        search_method.solve()
        solution_forward = search_method.get_solution()

        # get reverse solution
        search_method = search_method_type(problem_reverse)
        search_method.solve()
        solution_backward = search_method.get_solution()

        assert len(solution_forward) == len(solution_backward)

    # breadth first search
    search_method_test(sm.BreadthFirstSearch)

    # uniform cost search
    search_method_test(sm.UniformCostSearch)

    # iterative deepening search
    search_method_test(sm.IterativeDeepeningSearch)

    # A* search
    search_method_test(sm.AStarSearch)

    # bidirectional A* search
    search_method_test(sm.BidirectionalAStarSearch)
