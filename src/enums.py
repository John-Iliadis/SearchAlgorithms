from enum import Enum


class GridElement(Enum):
    """This enum represents the possible grid elements for the robot navigation problem."""
    EMPTY_CELL = 0
    WALL = 1
    AGENT = 2
    TARGET = 3


class Action(Enum):
    """This enum represents the available actions for the robot navigation problem."""
    MOVE_UP = 0
    MOVE_LEFT = 1
    MOVE_DOWN = 2
    MOVE_RIGHT = 3

    def __repr__(self):
        return self.name
