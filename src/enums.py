from enum import Enum


class GridElement(Enum):
    """This enum represents the possible grid elements for the robot navigation problem."""
    EMPTY_CELL = 0
    WALL = 1
    AGENT = 2
    TARGET = 3


class Action(Enum):
    """This enum represents the available actions for the robot navigation problem."""
    up = 0
    left = 1
    down = 2
    right = 3

    def __repr__(self):
        return f"'{self.name}'"
