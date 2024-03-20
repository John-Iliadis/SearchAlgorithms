import copy
from typing import List
from problem import Problem
from enums import Action
from utils import *


# todo: maybe the agent position should be taken out of the grid
class RobotNavigationProblem(Problem):
    def __init__(self, filename: str):
        super().__init__()
        self.grid, self.initial, self.goal = parse_robot_nav_file(filename)

    def actions(self, state: list) -> List['Action']:
        available_actions = [Action.MOVE_UP, Action.MOVE_LEFT, Action.MOVE_DOWN, Action.MOVE_RIGHT]

        if state[0] == 0 or get_grid_value(self.grid, [state[0] - 1, state[1]]) == GridElement.WALL:
            available_actions.remove(Action.MOVE_LEFT)
        if state[1] == 0 or get_grid_value(self.grid, [state[0], state[1] - 1]) == GridElement.WALL:
            available_actions.remove(Action.MOVE_UP)
        if state[0] >= self.grid.width - 1 or get_grid_value(self.grid, [state[0] + 1, state[1]]) == GridElement.WALL:
            available_actions.remove(Action.MOVE_RIGHT)
        if state[1] >= self.grid.height - 1 or get_grid_value(self.grid, [state[0], state[1] + 1]) == GridElement.WALL:
            available_actions.remove(Action.MOVE_DOWN)

        return available_actions

    def result(self, state: list, action: 'Action') -> list:
        """Calculates and returns a new state by performing the given action on the given state.
        This method assumes that 'action' is a valid action."""
        result_state = copy.deepcopy(state)

        if action == Action.MOVE_UP:
            result_state[1] -= 1
        elif action == Action.MOVE_LEFT:
            result_state[0] -= 1
        elif action == Action.MOVE_DOWN:
            result_state[1] += 1
        elif action == Action.MOVE_RIGHT:
            result_state[0] += 1
        else:
            raise ValueError(f"RobotNavigationProblem.result: Invalid value for Action: {action}")

        return result_state

    def goal_test(self, state: list) -> bool:
        return state in self.goal

    def path_cost(self, c: int, state1: list, action: 'Action', state2: list) -> int:
        """Assuming action is a valid action, the cost of moving once is one."""
        return c + 1

    def value(self, state):
        pass
