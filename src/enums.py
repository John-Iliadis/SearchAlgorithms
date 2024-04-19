from enum import Enum


class GridElement(Enum):
    """This enum represents the possible grid elements for the robot navigation problem."""
    empty_cell = 0
    wall = 1
    agent = 2
    target = 3


class Direction(Enum):
    """In the robot navigation problem the agent can move in this four directions."""
    up = 0
    left = 1
    down = 2
    right = 3
