from problem import RobotNavigationProblem
from enums import Direction
from vector import Vector2i
from action import JumpAction
import search_methods as sm


def test_available_jump_actions():
    problem = RobotNavigationProblem('../data/jumping/test_available_jump_actions.txt', JumpAction())
    available_actions = problem.actions(problem.initial)

    assert len(available_actions) == 4

    available_actions.sort(key=lambda action: action.direction.value)

    jump_up = available_actions[0]
    jump_left = available_actions[1]
    jump_down = available_actions[2]
    jump_right = available_actions[3]

    # jump up
    assert jump_up.direction == Direction.up
    assert jump_up.magnitude == 6

    # jump left
    assert jump_left.direction == Direction.left
    assert jump_left.magnitude == 6

    # jump down
    assert jump_down.direction == Direction.down
    assert jump_down.magnitude == 6

    # jump right
    assert jump_right.direction == Direction.right
    assert jump_right.magnitude == 6


def test_both_step_and_jumps_actions_combined():
    problem = RobotNavigationProblem('../data/jumping/test_both_step_and_jump_actions_combined.txt', JumpAction())
    available_actions = problem.actions(problem.initial)

    assert len(available_actions) == 4

    available_actions.sort(key=lambda action: action.direction.value)

    jump_up = available_actions[0]
    step_left = available_actions[1]
    jump_down = available_actions[2]
    jump_right = available_actions[3]

    # jump up
    assert jump_up.direction == Direction.up
    assert jump_up.magnitude == 3

    # step left
    assert step_left.direction == Direction.left
    assert step_left.magnitude == 1

    # jump down
    assert jump_down.direction == Direction.down
    assert jump_down.magnitude == 3

    # jump right
    assert jump_right.direction == Direction.right
    assert jump_right.magnitude == 3


def test_result_state():
    problem = RobotNavigationProblem('../data/jumping/test_result_state.txt', JumpAction())
    available_actions = problem.actions(problem.initial)

    assert len(available_actions) == 4

    available_actions.sort(key=lambda action: action.direction.value)

    jump_up = available_actions[0]
    jump_left = available_actions[1]
    jump_down = available_actions[2]
    jump_right = available_actions[3]

    # jump up
    result = problem.result(problem.initial, jump_up)
    assert result == Vector2i(6, 0)

    # jump left
    result = problem.result(problem.initial, jump_left)
    assert result == Vector2i(0, 6)

    # jump down
    result = problem.result(problem.initial, jump_down)
    assert result == Vector2i(6, 12)

    # jump right
    result = problem.result(problem.initial, jump_right)
    assert result == Vector2i(12, 6)


def test_finding_blocked_goal():
    problem = RobotNavigationProblem("../data/jumping/test_finding_blocked_goal.txt", JumpAction())

    def search_method_test(method_type):
        search_method = method_type(problem)
        search_method.solve()
        assert search_method.is_solved()

    # breadth first search
    search_method_test(sm.BreadthFirstSearch)

    # depth first search
    search_method_test(sm.DepthFirstSearch)

    # iterative deepening search
    search_method_test(sm.IterativeDeepeningSearch)

    # uniform cost search
    search_method_test(sm.UniformCostSearch)

    # greedy best first search
    search_method_test(sm.GreedyBestFirstSearch)

    # A* search
    search_method_test(sm.AStarSearch)

    # bidirectional A*
    search_method_test(sm.BidirectionalAStarSearch)


def test_no_goal_reachable():
    problem = RobotNavigationProblem("../data/jumping/test_no_goal_reachable.txt", JumpAction())

    def search_method_test(method_type):
        search_method = method_type(problem)
        search_method.solve()
        assert not search_method.is_solved()

    # breadth first search
    search_method_test(sm.BreadthFirstSearch)

    # depth first search
    search_method_test(sm.DepthFirstSearch)

    # iterative deepening search
    search_method_test(sm.IterativeDeepeningSearch)

    # uniform cost search
    search_method_test(sm.UniformCostSearch)

    # greedy best first search
    search_method_test(sm.GreedyBestFirstSearch)

    # A* search
    search_method_test(sm.AStarSearch)

    # bidirectional A*
    search_method_test(sm.BidirectionalAStarSearch)


def test_optimal_solution():
    problem = RobotNavigationProblem("../data/jumping/test_optimal_solution.txt", JumpAction())

    def search_method_test(method_type):
        search_method = method_type(problem)
        search_method.solve()
        assert search_method.goal_node.path_cost == 15

    # uniform cost search
    search_method_test(sm.UniformCostSearch)

    # A* search
    search_method_test(sm.UniformCostSearch)

    # bidirectional A* search
    search_method_test(sm.UniformCostSearch)
