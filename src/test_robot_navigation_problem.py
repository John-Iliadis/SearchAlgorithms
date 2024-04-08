from problem import RobotNavigationProblem
from enums import Direction
from vector import Vector2i
from action import Action
import utils


def test_file_parsing():
    grid, initial, goals = utils.parse_robot_nav_file("../data/core/test_file_parsing.txt")

    # grid
    assert grid.width == 5
    assert grid.height == 5
    assert len(grid.data) == 5
    assert grid.data[0] == [1, 1, 0, 0, 3]
    assert grid.data[1] == [1, 1, 0, 3, 0]
    assert grid.data[2] == [0, 0, 1, 0, 0]
    assert grid.data[3] == [0, 2, 0, 1, 1]
    assert grid.data[4] == [0, 0, 0, 1, 1]

    # initial state
    assert initial == Vector2i(3, 1)

    # goal states
    assert len(goals) == 2
    assert Vector2i(0, 4) in goals
    assert Vector2i(1, 3) in goals


def test_out_of_bounds_action():
    # up
    problem = RobotNavigationProblem("../data/core/test_out_of_bounds_up.txt")
    actions = problem.actions(problem.initial)
    assert Direction.up not in actions

    # left
    problem = RobotNavigationProblem("../data/core/test_out_of_bounds_left.txt")
    actions = problem.actions(problem.initial)
    assert Direction.left not in actions

    # down
    problem = RobotNavigationProblem("../data/core/test_out_of_bounds_down.txt")
    actions = problem.actions(problem.initial)
    assert Direction.down not in actions

    # right
    problem = RobotNavigationProblem("../data/core/test_out_of_bounds_right.txt")
    actions = problem.actions(problem.initial)
    assert Direction.right not in actions


def test_wall_check():
    problem = RobotNavigationProblem('../data/core/test_wall_check.txt')
    actions = problem.actions(problem.initial)

    assert len(actions) == 0


def test_result_action():
    # up
    problem = RobotNavigationProblem('../data/core/test_result_action.txt')
    result_state = problem.result(problem.initial, Action(Direction.up, 1))
    assert result_state == Vector2i(1, 0)

    # up
    result_state = problem.result(problem.initial, Action(Direction.left, 1))
    assert result_state == Vector2i(0, 1)

    # up
    result_state = problem.result(problem.initial, Action(Direction.down, 1))
    assert result_state == Vector2i(1, 2)

    # up
    result_state = problem.result(problem.initial, Action(Direction.right, 1))
    assert result_state == Vector2i(2, 1)


def test_admissible_heuristic():
    problem = RobotNavigationProblem("../data/core/test_admissible_heuristic.txt")

    heuristic_result = utils.straight_line_distance(Vector2i(0, 0), problem.goal)
    assert heuristic_result <= 9

    heuristic_result = utils.straight_line_distance(Vector2i(0, 3), problem.goal)
    assert heuristic_result <= 6

    heuristic_result = utils.straight_line_distance(Vector2i(6, 2), problem.goal)
    assert heuristic_result <= 1

    heuristic_result = utils.straight_line_distance(Vector2i(6, 6), problem.goal)
    assert heuristic_result <= 3
