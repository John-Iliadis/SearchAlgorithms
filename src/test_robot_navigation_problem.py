from problem import RobotNavigationProblem
from enums import Direction
from vector import Vector2i


def test_out_of_bounds_up_check_top_left():
    problem = RobotNavigationProblem('../data/pos_top_left.txt')
    actions = problem.actions(problem.initial)

    assert Direction.right in actions and Direction.down in actions


def test_out_of_bounds_check_bottom_right():
    problem = RobotNavigationProblem('../data/pos_bottom_right.txt')
    actions = problem.actions(problem.initial)

    assert Direction.left in actions and Direction.up in actions


def test_wall_check():
    problem = RobotNavigationProblem('../data/wall_check.txt')
    actions = problem.actions(problem.initial)

    assert len(actions) == 0


def test_result_up():
    problem = RobotNavigationProblem('../data/navigation_problem_1.txt')
    result_state = problem.result(Vector2i(5, 2), Direction.up)

    assert result_state == Vector2i(5, 1)


def test_result_down():
    problem = RobotNavigationProblem('../data/navigation_problem_1.txt')
    result_state = problem.result(Vector2i(5, 2), Direction.down)

    assert result_state == Vector2i(5, 3)


def test_result_left():
    problem = RobotNavigationProblem('../data/navigation_problem_1.txt')
    result_state = problem.result(Vector2i(5, 2), Direction.left)

    assert result_state == Vector2i(4, 2)


def test_result_right():
    problem = RobotNavigationProblem('../data/navigation_problem_1.txt')
    result_state = problem.result(Vector2i(5, 2), Direction.right)

    assert result_state == Vector2i(6, 2)
