from problem import RobotNavigationProblem
from enums import Action
from vector import Vector2i


def test_out_of_bounds_up_check_top_left():
    problem = RobotNavigationProblem('../data/pos_top_left.txt')
    actions = problem.actions(problem.initial)

    assert Action.right in actions and Action.down in actions


def test_out_of_bounds_check_bottom_right():
    problem = RobotNavigationProblem('../data/pos_bottom_right.txt')
    actions = problem.actions(problem.initial)

    assert Action.left in actions and Action.up in actions


def test_wall_check():
    problem = RobotNavigationProblem('../data/wall_check.txt')
    actions = problem.actions(problem.initial)

    assert len(actions) == 0


def test_result_up():
    problem = RobotNavigationProblem('../data/file.txt')
    result_state = problem.result(Vector2i(5, 2), Action.up)

    assert result_state == Vector2i(5, 1)


def test_result_down():
    problem = RobotNavigationProblem('../data/file.txt')
    result_state = problem.result(Vector2i(5, 2), Action.down)

    assert result_state == Vector2i(5, 3)


def test_result_left():
    problem = RobotNavigationProblem('../data/file.txt')
    result_state = problem.result(Vector2i(5, 2), Action.left)

    assert result_state == Vector2i(4, 2)


def test_result_right():
    problem = RobotNavigationProblem('../data/file.txt')
    result_state = problem.result(Vector2i(5, 2), Action.right)

    assert result_state == Vector2i(6, 2)
