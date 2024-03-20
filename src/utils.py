from enums import GridElement
from typing import Tuple, List
from grid import Grid
from vector import Vector2i
import numpy


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

    grid = numpy.full((grid_size.x, grid_size.y), GridElement.EMPTY_CELL.value).transpose().tolist()
    assign_grid_value(grid, start_pos, GridElement.AGENT.value)

    for target in targets:
        assign_grid_value(grid, target, GridElement.TARGET.value)

    for wall in walls:
        for x in range(wall[2]):
            for y in range(wall[3]):
                assign_grid_value(grid, Vector2i(wall[0] + x, wall[1] + y), GridElement.WALL.value)

    return Grid(grid, grid_size.y, grid_size.x), start_pos, targets


def print_grid(grid: 'Grid'):
    print(numpy.array(grid.data).transpose())


def get_grid_value(grid: 'Grid', position: 'Vector2i') -> 'GridElement':
    return GridElement(grid.data[position.x][position.y])