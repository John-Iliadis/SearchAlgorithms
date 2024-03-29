import copy
import utils
import numpy
from enums import Direction, GridElement
from typing import List
from vector import Vector2i


class Problem:
    """The abstract class for a formal problem. You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial=None, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        raise NotImplementedError

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2. If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        raise NotImplementedError

    def value(self, state):
        """For optimization problems, each state has a value. Hill Climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError

    def heuristic(self, state):
        raise NotImplementedError


class RobotNavigationProblem(Problem):
    def __init__(self, filename: str):
        super().__init__()
        self.grid, self.initial, self.goal = utils.parse_robot_nav_file(filename)

    def actions(self, state: 'Vector2i') -> List['Direction']:
        available_actions = [Direction.up, Direction.left, Direction.down, Direction.right]

        if state.x == 0 or self.grid.data[state.x - 1][state.y] == GridElement.WALL.value:
            available_actions.remove(Direction.left)
        if state.y == 0 or self.grid.data[state.x][state.y - 1] == GridElement.WALL.value:
            available_actions.remove(Direction.up)
        if state.x >= self.grid.width - 1 or self.grid.data[state.x + 1][state.y] == GridElement.WALL.value:
            available_actions.remove(Direction.right)
        if state.y >= self.grid.height - 1 or self.grid.data[state.x][state.y + 1] == GridElement.WALL.value:
            available_actions.remove(Direction.down)

        return available_actions

    def result(self, state: 'Vector2i', action: 'Direction') -> 'Vector2i':
        """Calculates and returns a new state by performing the given action on the given state.
        This method assumes that 'action' is a valid action."""
        result_state = copy.deepcopy(state)

        if action == Direction.up:
            result_state.y -= 1
        elif action == Direction.left:
            result_state.x -= 1
        elif action == Direction.down:
            result_state.y += 1
        elif action == Direction.right:
            result_state.x += 1
        else:
            raise ValueError(f"RobotNavigationProblem.result: Invalid value for Action: {action}")

        return result_state

    def goal_test(self, state: 'Vector2i') -> bool:
        return state in self.goal

    def path_cost(self, c: int, state1: 'Vector2i', action: 'Direction', state2: 'Vector2i') -> int:
        """Assuming action is a valid action, the cost of moving once is one."""
        return c + 1

    def heuristic(self, state: 'Vector2i'):
        """Returns the straight line distance between the given state and the closest goal state."""
        sld = numpy.inf

        for goal in self.goal:
            sld = min(sld, numpy.hypot(goal.x - state.x, goal.y - state.y))

        return sld
