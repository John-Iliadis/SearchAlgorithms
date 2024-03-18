from typing import Union
from collections import deque
from node import Node


def breadth_first_search(problem: 'Problem') -> Union['Node', None]:
    frontier = deque([Node(problem.initial)])

    while frontier:
        node = frontier.popleft()

        if problem.goal_test(node.state):
            return node

        frontier.extend(node.expand(problem))

    return None


def depth_first_search(problem: 'Problem') -> Union['Node', None]:
    frontier = [Node(problem.initial)]

    while frontier:
        node = frontier.pop()

        if problem.goal_test(node.state):
            return node

        frontier.extend(node.expand(problem))

    return None
