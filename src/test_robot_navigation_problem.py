from problem import RobotNavigationProblem
from enums import Direction
from vector import Vector2i


def test_out_of_bounds_action():
    # up
    problem = RobotNavigationProblem("../data/test_out_of_bounds_up.txt")
    actions = problem.actions(problem.initial)
    assert Direction.up not in actions

    # left
    problem = RobotNavigationProblem("../data/test_out_of_bounds_left.txt")
    actions = problem.actions(problem.initial)
    assert Direction.left not in actions

    # down
    problem = RobotNavigationProblem("../data/test_out_of_bounds_down.txt")
    actions = problem.actions(problem.initial)
    assert Direction.down not in actions

    # right
    problem = RobotNavigationProblem("../data/test_out_of_bounds_right.txt")
    actions = problem.actions(problem.initial)
    assert Direction.right not in actions


def test_wall_check():
    problem = RobotNavigationProblem('../data/test_wall_check.txt')
    actions = problem.actions(problem.initial)

    assert len(actions) == 0


def test_result_action():
    # up
    problem = RobotNavigationProblem('../data/test_result_action.txt')
    result_state = problem.result(problem.initial, Direction.up)
    assert result_state == Vector2i(1, 0)

    # up
    result_state = problem.result(problem.initial, Direction.left)
    assert result_state == Vector2i(0, 1)

    # up
    result_state = problem.result(problem.initial, Direction.down)
    assert result_state == Vector2i(1, 2)

    # up
    result_state = problem.result(problem.initial, Direction.right)
    assert result_state == Vector2i(2, 1)


def test_admissible_heuristic():
    problem = RobotNavigationProblem("../data/test_admissible_heuristic.txt")

    heuristic_result = problem.heuristic(Vector2i(0, 0))
    assert heuristic_result <= 9

    heuristic_result = problem.heuristic(Vector2i(0, 3))
    assert heuristic_result <= 6

    heuristic_result = problem.heuristic(Vector2i(6, 2))
    assert heuristic_result <= 1

    heuristic_result = problem.heuristic(Vector2i(6, 6))
    assert heuristic_result <= 3
