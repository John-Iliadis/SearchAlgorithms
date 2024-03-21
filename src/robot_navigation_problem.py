import copy
import math
from typing import List
from problem import Problem
from enums import Action
from vector import Vector2i
from numpy import inf
from utils import *


# todo: maybe the agent position should be taken out of the grid
class RobotNavigationProblem(Problem):
    def __init__(self, filename: str):
        super().__init__()
        self.grid, self.initial, self.goal = parse_robot_nav_file(filename)

    def actions(self, state: 'Vector2i') -> List['Action']:
        available_actions = [Action.MOVE_UP, Action.MOVE_LEFT, Action.MOVE_DOWN, Action.MOVE_RIGHT]

        if state.x == 0 or get_grid_value(self.grid, Vector2i(state.x - 1, state.y)) == GridElement.WALL:
            available_actions.remove(Action.MOVE_LEFT)
        if state.y == 0 or get_grid_value(self.grid, Vector2i(state.x, state.y - 1)) == GridElement.WALL:
            available_actions.remove(Action.MOVE_UP)
        if state.x >= self.grid.width - 1 or get_grid_value(self.grid, Vector2i(state.x + 1, state.y)) == GridElement.WALL:
            available_actions.remove(Action.MOVE_RIGHT)
        if state.y >= self.grid.height - 1 or get_grid_value(self.grid, Vector2i(state.x, state.y + 1)) == GridElement.WALL:
            available_actions.remove(Action.MOVE_DOWN)

        return available_actions

    def result(self, state: 'Vector2i', action: 'Action') -> 'Vector2i':
        """Calculates and returns a new state by performing the given action on the given state.
        This method assumes that 'action' is a valid action."""
        result_state = copy.deepcopy(state)

        if action == Action.MOVE_UP:
            result_state.y -= 1
        elif action == Action.MOVE_LEFT:
            result_state.x -= 1
        elif action == Action.MOVE_DOWN:
            result_state.y += 1
        elif action == Action.MOVE_RIGHT:
            result_state.x += 1
        else:
            raise ValueError(f"RobotNavigationProblem.result: Invalid value for Action: {action}")

        return result_state

    def goal_test(self, state: 'Vector2i') -> bool:
        return state in self.goal

    def path_cost(self, c: int, state1: 'Vector2i', action: 'Action', state2: 'Vector2i') -> int:
        """Assuming action is a valid action, the cost of moving once is one."""
        return c + 1

    def heuristic(self, state: 'Vector2i'):
        """Returns the straight line distance between the given state and the closest goal state."""
        sld = inf

        for goal in self.goal:
            sld = min(math.sqrt((goal.x - state.x)**2 + (goal.y - state.y)**2), sld)

        return sld
