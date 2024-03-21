import copy
import sys
from typing import Union, Tuple, Callable
from collections import deque
from node import Node
from vector import Vector2i
from priority_queue import PriorityQueue
from utils import *


# What needs to be calculated by the algorithms:
# - The goal node
# - The path from the root to the goal
# - The number of nodes created

# Considerations
# - Order of expansion: up, left, down, right
# - When all is equal, nodes should be expanded based on chronological order


# ______________________________________________________________________________
# Uninformed Search

def breadth_first_search(problem: 'Problem') -> Union[Tuple['Node', int], None]:
    frontier = deque([Node(problem.initial)])
    expanded = set()
    nodes_created = 1

    while frontier:
        node = frontier.popleft()

        if problem.goal_test(node.state):
            return node, nodes_created

        expanded.add(node.state)

        child_nodes = node.expand(problem)
        nodes_created += len(child_nodes)

        for child_node in child_nodes:
            if child_node.state not in expanded:
                frontier.append(child_node)
    return None


def depth_first_search(problem: 'Problem') -> Union[Tuple['Node', int], None]:
    frontier = [Node(problem.initial)]
    expanded = set()
    nodes_created = 1

    while frontier:
        node = frontier.pop()

        if problem.goal_test(node.state):
            return node, nodes_created

        expanded.add(node.state)

        child_nodes = node.expand(problem)
        child_nodes.reverse()
        nodes_created += len(child_nodes)

        for child_node in child_nodes:
            if child_node.state not in expanded:
                frontier.append(child_node)
    return None


def depth_limited_search(problem: 'Problem', depth_limit: int) -> Union[Tuple['Node', int], None]:
    frontier = [Node(problem.initial)]
    expanded = set()
    nodes_created = 1

    while frontier:
        node = frontier.pop()

        if problem.goal_test(node.state):
            return node, nodes_created
        elif node.depth > depth_limit:
            return None

        expanded.add(node.state)

        child_nodes = node.expand(problem)
        child_nodes.reverse()
        nodes_created += len(child_nodes)

        for child_node in child_nodes:
            if child_node.state not in expanded:
                frontier.append(child_node)
    return None


def iterative_deepening_search(problem: 'Problem') -> Union[Tuple['Node', int], None]:
    for depth in range(sys.maxsize):
        result = depth_limited_search(problem, depth)
        if result is not None:
            return result


def uniform_cost_search(problem: 'Problem') -> Union[Tuple['Node', int], None]:
    def f(node): return node.path_cost
    return best_first_search(problem, f)


# ______________________________________________________________________________
# Informed Search

def best_first_search(problem: 'Problem', evaluation_function: 'Callable', tiebreaker_function=None) -> Union[Tuple['Node', int], None]:
    frontier = PriorityQueue(evaluation_function, tiebreaker_function)
    frontier.append(Node(problem.initial))
    nodes_created = 1

    expanded = set()

    while frontier:
        node = frontier.pop()

        if problem.goal_test(node.state):
            return node, nodes_created

        expanded.add(node.state)

        child_nodes = node.expand(problem)
        nodes_created += len(child_nodes)

        for child in child_nodes:
            if child.state not in expanded:
                frontier.append(child)

    return None


def greedy_best_first_search(problem: 'Problem') -> Union[Tuple['Node', int], None]:
    def f(node): return problem.heuristic(node.state)
    def g(node): return get_action_value(node)

    return best_first_search(problem, f, g)


def a_star_search(problem: 'Problem') -> Union[Tuple['Node', int], None]:
    def f(node): return node.path_cost + problem.heuristic(node.state)
    def g(node): return get_action_value(node)

    return best_first_search(problem, f, g)

