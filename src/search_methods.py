import sys
from collections import deque
import numpy
from node import Node
from priority_queue import PriorityQueue
from problem import Problem, RobotNavigationProblem
import utils
from typing import Tuple


class SearchMethod:
    def __init__(self, problem):
        self.problem = problem
        self.goal_node = None
        self.method_name = None
        self.nodes_created = 0

    def solve(self):
        raise NotImplementedError

    def is_solved(self) -> bool:
        return self.goal_node is not None

    def get_solution(self) -> list:
        return self.goal_node.solution()

    def print_solution(self):
        if self.is_solved():
            print(f"{self.goal_node} {self.nodes_created}")
            self.problem.action_strategy.print_actions(self.get_solution())
        else:
            print(f"No goal is reachable; {self.nodes_created}")


# ______________________________________________________________________________
# Uninformed Search

class BreadthFirstSearch(SearchMethod):
    def __init__(self, problem: 'Problem'):
        super().__init__(problem)
        self.method_name = "BFS"

    def solve(self):
        frontier = deque([Node(self.problem.initial)])
        expanded = set()
        self.nodes_created = 1

        while frontier:
            node = frontier.popleft()

            if self.problem.goal_test(node.state):
                self.goal_node = node
                return

            expanded.add(node.state)

            for child_node in node.expand(self.problem):
                if child_node.state not in expanded:
                    frontier.append(child_node)
                    self.nodes_created += 1


class DepthFirstSearch(SearchMethod):
    def __init__(self, problem: 'Problem'):
        super().__init__(problem)
        self.method_name = "DFS"

    def solve(self):
        frontier = [Node(self.problem.initial)]
        expanded = set()
        self.nodes_created = 1

        while frontier:
            node = frontier.pop()

            if self.problem.goal_test(node.state):
                self.goal_node = node
                return

            expanded.add(node.state)

            for child_node in list(reversed(node.expand(self.problem))):
                if child_node.state not in expanded:
                    frontier.append(child_node)
                    self.nodes_created += 1


class DepthLimitedSearch(SearchMethod):
    def __init__(self, problem: 'Problem', depth_limit: int = 20):
        super().__init__(problem)
        self.method_name = "Depth Limited Search"
        self.depth_limit = depth_limit
        self.expanded = set()

    def solve(self):
        frontier = [Node(self.problem.initial)]
        self.nodes_created = 1

        while frontier:
            node = frontier.pop()

            if self.problem.goal_test(node.state):
                self.goal_node = node
                return

            self.expanded.add(node.state)

            if node.depth >= self.depth_limit:
                continue

            for child_node in list(reversed(node.expand(self.problem))):
                if not self.is_cycle(child_node):
                    frontier.append(child_node)
                    self.nodes_created += 1

    def is_cycle(self, node: 'Node') -> bool:
        """Checks for a cycle by going up the node's parent chain."""
        path = node.path()
        path.reverse()

        for parent in path[1:]:
            if node.state == parent.state:
                return True
        return False


class IterativeDeepeningSearch(SearchMethod):
    def __init__(self, problem: 'Problem'):
        super().__init__(problem)
        self.method_name = "CUS1"

    def solve(self):
        # variable indicating how many nodes where expanded in a depth limited search. This is used for stopping
        # iterative deepening search from looping infinitely when no solution is available.
        prev_expanded_count = numpy.inf

        for depth in range(sys.maxsize):
            # solve using depth limited
            depth_limited_search = DepthLimitedSearch(self.problem, depth)
            depth_limited_search.solve()
            self.nodes_created += depth_limited_search.nodes_created

            current_expanded_count = len(depth_limited_search.expanded)

            if depth_limited_search.is_solved():
                # solution found
                self.goal_node = depth_limited_search.goal_node
                return
            elif prev_expanded_count == current_expanded_count:
                # if no new nodes were expanded, that means all the available nodes have been reached and there
                # is no solution available
                return
            else:
                prev_expanded_count = current_expanded_count


class BestFirstSearch(SearchMethod):
    def __init__(self, problem: 'Problem', evaluation_function, tie_breaker=None):
        super().__init__(problem)
        self.method_name = "Best First Search"
        self.f = evaluation_function
        self.g = tie_breaker

    def solve(self):
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

            for child_node in node.expand(self.problem):
                if child_node.state not in expanded:
                    frontier.append(child_node)
                    self.nodes_created += 1


class UniformCostSearch(BestFirstSearch):
    def __init__(self, problem: 'Problem'):
        def f(node): return node.path_cost
        super().__init__(problem, f, get_tie_breaker_function())
        self.method_name = "Uniform Cost Search"


# ______________________________________________________________________________
# Informed Search


def get_sld_function(goal_states: list):
    return lambda node: utils.straight_line_distance(node.state, goal_states)


def get_a_star_heuristic(goal_states: list):
    return lambda node: node.path_cost + utils.straight_line_distance(node.state, goal_states)


def get_tie_breaker_function():
    return lambda node: utils.get_direction_value(node)


class GreedyBestFirstSearch(BestFirstSearch):
    def __init__(self, problem: 'Problem'):
        super().__init__(problem, get_sld_function(problem.goal), get_tie_breaker_function())
        self.method_name = "GBFS"


class AStarSearch(BestFirstSearch):
    def __init__(self, problem: 'Problem'):
        super().__init__(problem, get_a_star_heuristic(problem.goal), get_tie_breaker_function())
        self.method_name = "A*"


class BidirectionalAStarSearch(BestFirstSearch):
    def __init__(self, problem: 'RobotNavigationProblem'):
        if not isinstance(problem, RobotNavigationProblem):
            raise TypeError("This Bidirectional A* search class is designed specifically for the robot navigation problem")

        super().__init__(problem, get_a_star_heuristic(problem.goal), get_tie_breaker_function())
        self.method_name = "CUS2"
        self.solution = None  # solution is a list of actions
        self.lowest_cost_so_far = numpy.inf

    def solve(self):
        frontier_forward = PriorityQueue(get_a_star_heuristic(self.problem.goal), get_tie_breaker_function())
        frontier_forward.append(Node(self.problem.initial))
        self.nodes_created = 1

        frontiers_backward = []

        # calculate backwards frontiers
        for goal_state in self.problem.goal:
            pq = PriorityQueue(get_a_star_heuristic([self.problem.initial]), get_tie_breaker_function())
            frontiers_backward.append(pq)
            frontiers_backward[-1].append(Node(goal_state))
            self.nodes_created += 1

        # only one of each expanded dicts
        expanded_forward = {}
        expanded_backward = {}

        while frontier_forward and any(frontier for frontier in frontiers_backward):
            cheapest_backward_frontier = min(frontiers_backward, key=lambda frontier: self.get_f_value(frontier))

            # the combined f values of the priority node in the forward frontier and the cheapest backward frontier
            combined_f_values = self.get_f_value(frontier_forward) + self.get_f_value(cheapest_backward_frontier)

            if combined_f_values < self.lowest_cost_so_far:
                if self.top_node_has_min_f_value(frontier_forward, cheapest_backward_frontier):
                    self.proceed('forward', frontier_forward, cheapest_backward_frontier, expanded_forward, expanded_backward)
                else:
                    self.proceed('backward', cheapest_backward_frontier, frontier_forward, expanded_backward, expanded_forward)
            else:
                break

    def proceed(self, direction: str, frontier_a, frontier_b, expanded_a, expanded_b):
        """Expands a node and checks for a solution."""
        evaluation_function_a = frontier_a.f
        evaluation_function_b = frontier_b.f

        node = frontier_a.pop()
        expanded_a[node.state] = node

        # check for a solution
        if node.state in expanded_b:
            new_solution, new_goal_node = self.make_solution(direction, node, expanded_b[node.state])
            if self.path_cost(new_solution) < self.path_cost(self.solution):
                self.solution = new_solution
                self.goal_node = new_goal_node
                self.lowest_cost_so_far = evaluation_function_a(node) + evaluation_function_b(node)

        # extend frontier
        for child_node in node.expand(self.problem):
            if child_node.state not in expanded_a or child_node.path_cost < expanded_a[child_node.state].path_cost:
                frontier_a.append(child_node)
                self.nodes_created += 1

    def get_f_value(self, frontier):
        """Returns the f value of the item with the highest priority in the priority queue."""
        if frontier:
            return frontier.f(frontier.peak())
        return numpy.inf

    def make_solution(self, direction: str, node_a: 'Node', node_b: 'Node') -> Tuple[list, 'Node']:
        """The solution is a list of actions that combines node_a and node_b. The actions of the goal node are
        reversed, depending on the direction."""
        solution = []
        goal_node = None

        if direction == 'forward':
            actions_a = node_a.solution()
            actions_b = [utils.reverse_direction(action) for action in node_b.solution()]
            actions_b.reverse()
            solution = actions_a + actions_b
            goal_node = node_b.path()[0]
        elif direction == 'backward':
            actions_a = [utils.reverse_direction(action) for action in node_a.solution()]
            actions_a.reverse()
            actions_b = node_b.solution()
            solution = actions_b + actions_a
            goal_node = node_a.path()[0]
        else:
            raise RuntimeError("bidirectional_best_first_search::make_solution: 'direction' has invalid value")

        return solution, goal_node

    def top_node_has_min_f_value(self, frontier_a, frontier_b) -> bool:
        """Returns true if frontier_a's priority node has a smaller f value than frontier_b's priority node."""
        node_a = frontier_a.peak()
        node_b = frontier_b.peak()

        f_value_a = frontier_a.f(node_a)
        f_value_b = frontier_b.f(node_b)

        return f_value_a <= f_value_b

    def path_cost(self, solution: list):
        """Calculates the total cost of a solution."""
        if solution is None:
            return numpy.inf

        total = 0
        for action in solution:
            total += action.magnitude

        return total

    def get_solution(self):
        return self.solution
