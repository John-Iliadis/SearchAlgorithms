from enum import Enum


class Position(Enum):
    EMPTY_CELL = 0
    WALL = 1
    AGENT = 2
    TARGET = 3
