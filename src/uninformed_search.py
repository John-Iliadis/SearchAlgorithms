from typing import Union, Tuple
from collections import deque
from node import Node
from vector import Vector2i

# What needs to be calculated by the algorithms:
# - The goal node
# - The path from the root to the goal
# - The number of nodes created

# Considerations
# - Order of expansion: up, left, down, right
# - When all is equal, nodes should be expanded based on chronological order


def breadth_first_search(problem: 'Problem') -> Union[Tuple['Node', int], None]:
    frontier = deque([Node(problem.initial)])
    expanded = set()
    nodes_created = 1

    while frontier:
        node = frontier.popleft()

        expanded.add(node.state)

        if problem.goal_test(node.state):
            return node, nodes_created

        child_nodes = node.expand(problem)
        nodes_created += len(child_nodes)

        for child_node in child_nodes:
            if child_node.state not in expanded:
                frontier.append(child_node)


def depth_first_search(problem: 'Problem') -> Union[Tuple['Node', int], None]:
    frontier = [Node(problem.initial)]
    expanded = set()
    nodes_created = 1

    while frontier:
        node = frontier.pop()

        if node.state == Vector2i(0, 2):
            breakpoint()

        expanded.add(node.state)

        if problem.goal_test(node.state):
            return node, nodes_created

        child_nodes = node.expand(problem)
        child_nodes.reverse()
        nodes_created += len(child_nodes)

        for child_node in child_nodes:
            if child_node.state not in expanded:
                frontier.append(child_node)


def bidirectional_search(problem: 'Problem') -> Union[Tuple['Node', int], None]:
    pass
