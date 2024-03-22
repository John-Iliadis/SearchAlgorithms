import copy
import sys
from typing import Union, Tuple, Callable
from collections import deque
import numpy
from node import Node
from vector import Vector2i
from priority_queue import PriorityQueue
from problem import Problem
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


def make_solution(direction: str, node_a: 'Node', node_b: 'Node') -> list:
    solution = []

    if direction == 'forward':
        actions_a = node_a.solution()
        actions_b = [reverse_action(action) for action in node_b.solution()]
        solution = actions_a + actions_b
    elif direction == 'backward':
        actions_a = [reverse_action(action) for action in node_b.solution()]
        actions_b = node_b.solution()
        solution = actions_b + actions_a
    else:
        raise RuntimeError("bidirectional_best_first_search::make_solution: 'direction' has invalid value")

    return solution


def step(direction: str, problem_a: 'Problem', frontier_a: 'PriorityQueue', frontier_b: 'PriorityQueue', expanded_a, expanded_b, solution, lowest_cost_so_far):
    node = frontier_a.pop()
    child_nodes = node.expand(problem_a)

    for child in child_nodes:
        state = child.state
        if state not in expanded_a or child.path_cost < expanded_a[state].path_cost:
            expanded_a[state] = child
            frontier_a.append(child)
            if state in expanded_b:
                new_solution = make_solution(direction, child, expanded_b[state])
                lowest_cost_so_far = min(lowest_cost_so_far, frontier_a.f(child) + frontier_b.f(child))
                if solution is None or len(new_solution) < len(solution):
                    return new_solution, lowest_cost_so_far
    return solution, lowest_cost_so_far


def top_node_has_min_f_value(frontier_a: 'PriorityQueue', frontier_b: 'PriorityQueue') -> bool:
    node_a = frontier_a.peak()
    node_b = frontier_b.peak()

    f_value_a = frontier_a.f(node_a)
    f_value_b = frontier_b.f(node_b)

    return f_value_a <= f_value_b


def get_f_value(frontier: 'PriorityQueue'):
    return frontier.f(frontier.peak())


def bidirectional_a_star(problem: 'Problem'):
    problem_forward = copy.deepcopy(problem)
    problem_backward = copy.deepcopy(problem)
    problem_backward.initial, problem_backward.goal = problem_backward.goal[0], [problem_backward.initial]

    def f_forward(node): return node.path_cost + problem_forward.heuristic(node.state)
    def f_backward(node): return node.path_cost + problem_backward.heuristic(node.state)
    def g(node): return get_action_value(node)

    frontier_forward = PriorityQueue(f_forward, g)
    frontier_backward = PriorityQueue(f_backward, g)

    frontier_forward.append(Node(problem_forward.initial))
    frontier_backward.append(Node(problem_backward.initial))

    expanded_forward, expanded_backward = {}, {}

    lowest_cost_so_far = numpy.inf
    solution = None

    while frontier_forward and frontier_backward:
        combined_f_values = get_f_value(frontier_forward) + get_f_value(frontier_backward)

        if combined_f_values < lowest_cost_so_far:
            if top_node_has_min_f_value(frontier_forward, frontier_backward):
                solution, lowest_cost_so_far = step('forward', problem_forward, frontier_forward, frontier_backward, expanded_forward, expanded_backward, solution, lowest_cost_so_far)
            else:
                solution, lowest_cost_so_far = step('backward', problem_backward, frontier_backward, frontier_forward, expanded_backward, expanded_forward, solution, lowest_cost_so_far)
        else:
            break
    return solution
