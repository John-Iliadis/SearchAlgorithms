from position import Position
import numpy


def string_to_vectori(string: str, strip_chars: str, delim: str) -> list:
    """Converts a string to an integer vector: "(x, y, z)" -> [x, y, z]"""
    return [int(x) for x in string.strip(strip_chars).split(delim)]


def assign_grid_value(grid: list, position: list, value: 'Position'):
    """Assigns a value to a 2d grid. 'position' is a 2d vector with the index for the grid."""
    grid[position[0]][position[1]] = value


def parse_robot_nav_file(filename: str) -> list:
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
            walls.append(string_to_vectori(line, ' ()\n', ','))

    grid = numpy.full(grid_size, Position.EMPTY_CELL.value).transpose().tolist()
    assign_grid_value(grid, start_pos, Position.AGENT.value)

    for target in targets:
        assign_grid_value(grid, target, Position.TARGET.value)

    for wall in walls:
        for x in range(wall[2]):
            for y in range(wall[3]):
                assign_grid_value(grid, [wall[0] + x, wall[1] + y], Position.WALL.value)

    return grid


def print_grid(grid: list):
    print(numpy.array(grid).transpose())
