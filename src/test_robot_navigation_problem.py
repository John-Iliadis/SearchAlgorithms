from robot_navigation_problem import RobotNavigationProblem
from enums import Action
import utils


def test_out_of_bounds_up_check_top_left():
    problem = RobotNavigationProblem('../data/pos_top_left.txt')
    actions = problem.actions(problem.initial)

    assert Action.MOVE_RIGHT in actions and Action.MOVE_DOWN in actions


def test_out_of_bounds_check_bottom_right():
    problem = RobotNavigationProblem('../data/pos_bottom_right.txt')
    actions = problem.actions(problem.initial)

    assert Action.MOVE_LEFT in actions and Action.MOVE_UP in actions


def test_wall_check():
    problem = RobotNavigationProblem('../data/wall_check.txt')
    actions = problem.actions(problem.initial)

    assert len(actions) == 0


def test_result_up():
    problem = RobotNavigationProblem('../data/file.txt')
    result_state = problem.result([5, 2], Action.MOVE_UP)

    assert result_state == [5, 1]


def test_result_down():
    problem = RobotNavigationProblem('../data/file.txt')
    result_state = problem.result([5, 2], Action.MOVE_DOWN)

    assert result_state == [5, 3]


def test_result_left():
    problem = RobotNavigationProblem('../data/file.txt')
    result_state = problem.result([5, 2], Action.MOVE_LEFT)

    assert result_state == [4, 2]


def test_result_right():
    problem = RobotNavigationProblem('../data/file.txt')
    result_state = problem.result([5, 2], Action.MOVE_RIGHT)

    assert result_state == [6, 2]
