import copy
import sys
from typing import Union, Tuple, Callable
from collections import deque
import numpy
from node import Node
from vector import Vector2i
from priority_queue import PriorityQueue
from problem import Problem, RobotNavigationProblem
import utils


class SearchMethod:
    def __init__(self, problem: 'Problem' = None):
        self.goal_node = None
        self.nodes_created = 0
        self.method_name = None
        self.problem = problem

    def solve(self):
        raise NotImplementedError

    def is_solved(self):
        return self.goal_node is not None

    def solution(self):
        return self.goal_node.solution()

    def clear(self):
        if self.is_solved():
            self.goal_node = None
            self.nodes_created = 0


# ______________________________________________________________________________
# Uninformed Search

class BreadthFirstSearch(SearchMethod):
    def __init__(self, problem: 'Problem'):
        super().__init__(problem)
        self.method_name = "Breadth First Search"

    def solve(self):
        super().clear()

        frontier = deque([Node(self.problem.initial)])
        expanded = set()
        self.nodes_created = 1

        while frontier:
            node = frontier.popleft()

            if self.problem.goal_test(node.state):
                self.goal_node = node
                return

            expanded.add(node.state)

            child_nodes = node.expand(self.problem)
            self.nodes_created += len(child_nodes)

            for child_node in child_nodes:
                if child_node.state not in expanded:
                    frontier.append(child_node)


class DepthFirstSearch(SearchMethod):
    def __init__(self, problem: 'Problem'):
        super().__init__(problem)
        self.method_name = "Depth First Search"

    def solve(self):
        super().clear()

        frontier = [Node(self.problem.initial)]
        expanded = set()
        self.nodes_created = 1

        while frontier:
            node = frontier.pop()

            if self.problem.goal_test(node.state):
                self.goal_node = node
                return

            expanded.add(node.state)

            child_nodes = node.expand(self.problem)
            child_nodes.reverse()
            self.nodes_created += len(child_nodes)

            for child_node in child_nodes:
                if child_node.state not in expanded:
                    frontier.append(child_node)


class DepthLimitedSearch(SearchMethod):
    def __init__(self, problem: 'Problem', depth_limit: int = 20):
        super().__init__(problem)
        self.method_name = "Depth Limited Search"
        self.depth_limit = depth_limit

    def solve(self):
        super().clear()

        frontier = [Node(self.problem.initial)]
        expanded = set()
        self.nodes_created = 1

        while frontier:
            node = frontier.pop()

            if self.problem.goal_test(node.state):
                self.goal_node = node
                return
            elif node.depth > self.depth_limit:
                return

            expanded.add(node.state)

            child_nodes = node.expand(self.problem)
            child_nodes.reverse()
            self.nodes_created += len(child_nodes)

            for child_node in child_nodes:
                if child_node.state not in expanded:
                    frontier.append(child_node)


class IterativeDeepeningSearch(SearchMethod):
    def __init__(self, problem: 'Problem'):
        super().__init__(problem)
        self.method_name = "Iterative Deepening Search"

    def solve(self):
        super().clear()

        for depth in range(sys.maxsize):
            depth_limited_search = DepthLimitedSearch(self.problem, depth)
            depth_limited_search.solve()
            self.nodes_created += depth_limited_search.nodes_created
            if depth_limited_search.is_solved():
                self.goal_node = depth_limited_search.goal_node
                return


class BestFirstSearch(SearchMethod):
    def __init__(self, problem: 'Problem', evaluation_function, tie_breaker=None):
        super().__init__(problem)
        self.method_name = "Best First Search"
        self.f = evaluation_function
        self.g = tie_breaker

    def solve(self):
        super().clear()

        frontier = PriorityQueue(self.f, self.g)
        frontier.append(Node(self.problem.initial))
        self.nodes_created = 1

        expanded = set()

        while frontier:
            node = frontier.pop()

            if self.problem.goal_test(node.state):
                self.goal_node = node
                return

            expanded.add(node.state)

            child_nodes = node.expand(self.problem)
            self.nodes_created += len(child_nodes)

            for child in child_nodes:
                if child.state not in expanded:
                    frontier.append(child)


class UniformCostSearch(BestFirstSearch):
    def __init__(self, problem: 'Problem'):
        def f(node): return node.path_cost
        super().__init__(problem, f)
        self.method_name = "Uniform Cost Search"


# ______________________________________________________________________________
# Informed Search

class GreedyBestFirstSearch(BestFirstSearch):
    def __init__(self, problem: 'Problem'):
        def f(node): return problem.heuristic(node.state)
        def g(node): return utils.get_action_value(node)
        super().__init__(problem, f, g)
        self.method_name = "Greedy Best First Search"


class AStarSearch(BestFirstSearch):
    def __init__(self, problem: 'Problem'):
        def f(node): return node.path_cost + problem.heuristic(node.state)
        def g(node): return utils.get_action_value(node)
        super().__init__(problem, f, g)
        self.method_name = "A* Search"


class BidirectionalAStarSearch(SearchMethod):
    def __init__(self, problem: 'RobotNavigationProblem'):
        if not isinstance(problem, RobotNavigationProblem):
            raise TypeError("This Bidirectional A* search class is designed specifically for the robot navigation problem")

        super().__init__()
        self.method_name = "Bidirectional A* Search"

        self.problem_forward = copy.deepcopy(problem)
        self.problem_backward = self.make_backward_problem(problem)

        self.solution = None  # solution is a list of actions
        self.lowest_cost_so_far = numpy.inf  # the current lowest cost of one of the solutions found

        self.f_forward = lambda node: node.path_cost + self.problem_forward.heuristic(node.state)
        self.f_backward = lambda node: node.path_cost + self.problem_backward.heuristic(node.state)
        self.g = lambda node: utils.get_action_value(node)

    def make_backward_problem(self, problem: 'RobotNavigationProblem'):
        """In a backward problem, the initial and goal states are swapped. """

        problem_backward = copy.deepcopy(problem)
        closest_state = problem_backward.goal[0]
        goal = problem.initial

        # find the goal state closest to the initial state before swapping
        for state in problem_backward.goal[1:]:
            if numpy.hypot(goal.x - state.x, goal.y - state.y) < numpy.hypot(goal.x - closest_state.x, goal.y - closest_state.y):
                closest_state = state

        problem_backward.initial, problem_backward.goal = closest_state, [problem_backward.initial]
        return problem_backward

    def solve(self):
        self.clear()

        frontier_forward = PriorityQueue(self.f_forward, self.g)
        frontier_backward = PriorityQueue(self.f_backward, self.g)

        frontier_forward.append(Node(self.problem_forward.initial))
        frontier_backward.append(Node(self.problem_backward.initial))

        expanded_forward, expanded_backward = {}, {}

        while frontier_forward and frontier_backward:
            combined_f_values = self.get_f_value(frontier_forward) + self.get_f_value(frontier_backward)

            if combined_f_values < self.lowest_cost_so_far:
                if self.top_node_has_min_f_value(frontier_forward, frontier_backward):
                    self.step('forward', self.problem_forward, frontier_forward, frontier_backward, expanded_forward, expanded_backward)
                else:
                    self.step('backward', self.problem_backward, frontier_backward, frontier_forward, expanded_backward, expanded_forward)
            else:
                break

    def step(self, direction: str, problem_a, frontier_a, frontier_b, expanded_a, expanded_b):
        """Expands a node in one direction and checks for a solution."""
        node = frontier_a.pop()
        state = node.state

        expanded_a[state] = node

        # solution found when expanded node is in the other direction's expanded list
        if node.state in expanded_b:
            # calculate new solution and lowest solution cost
            new_solution = self.make_solution(direction, node, expanded_b[state])
            self.lowest_cost_so_far = min(self.lowest_cost_so_far, frontier_a.f(node) + frontier_b.f(node))
            # if the new solution is cheaper, then it becomes the current solution
            if self.solution is None or len(new_solution) < len(self.solution):
                self.solution = new_solution

        # expand frontier
        child_nodes = node.expand(problem_a)
        self.nodes_created += len(child_nodes)

        for child in child_nodes:
            state = child.state
            if state not in expanded_a:
                frontier_a.append(child)

    def make_solution(self, direction: str, node_a, node_b):
        """The solution is a list of actions that combines node_a and node_b. The actions of the goal node are
        reversed, depending on the direction."""
        solution = []

        if direction == 'forward':
            actions_a = node_a.solution()
            actions_b = [utils.reverse_action(action) for action in node_b.solution()]
            solution = actions_a + actions_b
        elif direction == 'backward':
            actions_a = [utils.reverse_action(action) for action in node_a.solution()]
            actions_a.reverse()
            actions_b = node_b.solution()
            solution = actions_b + actions_a
        else:
            raise RuntimeError("bidirectional_best_first_search::make_solution: 'direction' has invalid value")

        return solution

    def top_node_has_min_f_value(self, frontier_a, frontier_b) -> bool:
        """Returns true if frontier_a's priority node has a smaller f value than frontier_b's priority node."""
        node_a = frontier_a.peak()
        node_b = frontier_b.peak()

        f_value_a = frontier_a.f(node_a)
        f_value_b = frontier_b.f(node_b)

        return f_value_a <= f_value_b

    def get_f_value(self, frontier):
        """Returns the f value of the item with the highest priority in the priority queue."""
        return frontier.f(frontier.peak())

    def clear(self):
        """Clears the results."""
        if self.solution is not None:
            self.nodes_created = 0
            self.solution = None
            self.lowest_cost_so_far = numpy.inf
