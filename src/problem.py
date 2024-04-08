import copy
import utils
import numpy
from enums import Direction
from typing import List
from vector import Vector2i
from action import Action, ActionStrategy, SingleStepAction


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


class RobotNavigationProblem(Problem):
    def __init__(self, filename: str, action_strategy: 'ActionStrategy' = SingleStepAction()):
        super().__init__()
        self.grid, self.initial, self.goal = utils.parse_robot_nav_file(filename)
        self.action_strategy = action_strategy

    def actions(self, state: 'Vector2i') -> List['Action']:
        return self.action_strategy.evaluate_actions(state, self.grid)

    def result(self, state: 'Vector2i', action: 'Action') -> 'Vector2i':
        """Calculates and returns a new state by performing the given action on the given state."""
        result_state = copy.deepcopy(state)

        if action.direction == Direction.up:
            result_state.y -= action.magnitude
        elif action.direction == Direction.left:
            result_state.x -= action.magnitude
        elif action.direction == Direction.down:
            result_state.y += action.magnitude
        elif action.direction == Direction.right:
            result_state.x += action.magnitude
        else:
            raise ValueError(f"RobotNavigationProblem.result: Invalid value for action")

        return result_state

    def goal_test(self, state: 'Vector2i') -> bool:
        return state in self.goal

    def path_cost(self, c: int, state1: 'Vector2i', action: 'Action', state2: 'Vector2i') -> int:
        return c + 2**(action.magnitude - 1)
