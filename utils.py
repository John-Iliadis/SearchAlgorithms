import numpy
from typing import Tuple, List
from grid import Grid
from vector import Vector2i
from enums import GridElement
from node import Node


def string_to_list(string: str, strip_chars: str, delim: str) -> List[int]:
    """Converts a string to an integer list."""
    return [int(x) for x in string.strip(strip_chars).split(delim)]


def string_to_vectori(string: str, strip_chars: str, delim: str) -> 'Vector2i':
    """Converts a string to a Vector2i."""
    vec = string_to_list(string, strip_chars, delim)
    return Vector2i(vec[0], vec[1])


def assign_grid_value(grid: list, position: 'Vector2i', value: 'GridElement'):
    """Assigns a value to a 2d grid. 'position' is a 2d vector with the index for the grid."""
    grid[position.x][position.y] = value


def parse_robot_nav_file(filename: str) -> Tuple['Grid', 'Vector2i', list]:
    """Parses a file for the robot navigation problem. Returns a 2d grid that consists of
    the agents initial position, the wall positions, and the target positions. Use
    print_grid() to check the result grid after parsing."""
    with open(filename, 'r') as file:
        grid_size = string_to_vectori(file.readline(), ' []\n', ',')
        start_pos = string_to_vectori(file.readline(), ' ()\n', ',')
        targets = []
        walls = []

        for pos in file.readline().split('|'):
            targets.append(string_to_vectori(pos, ' ()\n', ','))

        for line in file.readlines():
            walls.append(string_to_list(line, ' ()\n', ','))

    grid = numpy.full((grid_size.x, grid_size.y), GridElement.empty_cell.value).transpose().tolist()
    assign_grid_value(grid, start_pos, GridElement.agent.value)

    for target in targets:
        assign_grid_value(grid, target, GridElement.target.value)

    targets = list(set(targets))  # get rid of duplicate values

    for wall in walls:
        for x in range(wall[2]):
            for y in range(wall[3]):
                assign_grid_value(grid, Vector2i(wall[0] + x, wall[1] + y), GridElement.wall.value)

    return Grid(grid, grid_size.y, grid_size.x), start_pos, targets


def get_direction_value(node: 'Node'):
    if node.action is not None:
        return node.action.direction.value
    return 0


def straight_line_distance(state: 'Vector2i', goal_states: list):
    """Returns the straight line distance between the given state and the closest goal state."""
    sld = numpy.inf

    for goal in goal_states:
        sld = min(sld, numpy.hypot(goal.x - state.x, goal.y - state.y))

    return sld
