import copy
import sys
from collections import deque
import numpy
from node import Node
from priority_queue import PriorityQueue
from problem import Problem, RobotNavigationProblem
import utils


class SearchMethod:
    def __init__(self, problem: 'Problem' = None):
        self.problem = problem
        self.goal_node = None
        self.method_name = None
        self.nodes_created = 0

    def solve(self):
        raise NotImplementedError

    def is_solved(self):
        return self.goal_node is not None

    def solution(self):
        return self.goal_node.solution()

    def print_solution(self):
        if self.is_solved():
            print(f"{self.goal_node} {self.nodes_created}")
            utils.print_solution(self.goal_node.solution())
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

            child_nodes = node.expand(self.problem)
            child_nodes.sort(key=lambda n: n.action.value, reverse=True)

            for child_node in child_nodes:
                if child_node.state not in expanded:
                    frontier.append(child_node)
                    self.nodes_created += 1


class DepthLimitedSearch(SearchMethod):
    def __init__(self, problem: 'Problem', depth_limit: int = 20):
        super().__init__(problem)
        self.method_name = "Depth Limited Search"
        self.depth_limit = depth_limit

    def solve(self):
        frontier = [Node(self.problem.initial)]
        self.nodes_created = 1

        while frontier:
            node = frontier.pop()

            if self.problem.goal_test(node.state):
                self.goal_node = node
                return
            elif node.depth >= self.depth_limit:
                continue

            if not self.is_cycle(node):
                child_nodes = node.expand(self.problem)
                child_nodes.sort(key=lambda n: n.action.value, reverse=True)

                for child in child_nodes:
                    frontier.append(child)
                    self.nodes_created += 1

    def is_cycle(self, node: 'Node') -> bool:
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

            for child in node.expand(self.problem):
                if child.state not in expanded:
                    frontier.append(child)
                    self.nodes_created += 1


class UniformCostSearch(BestFirstSearch):
    def __init__(self, problem: 'Problem'):
        def f(node): return node.path_cost
        def g(node): return utils.get_direction_value(node)
        super().__init__(problem, f)
        self.method_name = "Uniform Cost Search"


# ______________________________________________________________________________
# Informed Search

class GreedyBestFirstSearch(BestFirstSearch):
    def __init__(self, problem: 'Problem'):
        def f(node): return problem.heuristic(node.state)
        def g(node): return utils.get_action_value(node)
        super().__init__(problem, f, g)
        self.method_name = "GBFS"


class AStarSearch(BestFirstSearch):
    def __init__(self, problem: 'Problem'):
        def f(node): return node.path_cost + problem.heuristic(node.state)
        def g(node): return utils.get_action_value(node)
        super().__init__(problem, f, g)
        self.method_name = "A*"


class BidirectionalAStarSearch(SearchMethod):
    def __init__(self, problem: 'RobotNavigationProblem'):
        if not isinstance(problem, RobotNavigationProblem):
            raise TypeError("This Bidirectional A* search class is designed specifically for the robot navigation problem")

        super().__init__()
        self.method_name = "CUS2"

        self.problem_forward = copy.deepcopy(problem)
        self.problems_backward = self.make_backward_problems(problem)  # list of backward problems

        self.solution = None  # solution is a list of actions
        self.lowest_cost_so_far = numpy.inf  # the current lowest cost of one of the solutions found

        self.g = lambda node: utils.get_action_value(node)

    def solve(self):
        frontier_forward = PriorityQueue(self.get_a_star_heuristic(self.problem_forward), self.g)
        frontier_forward.append(Node(self.problem_forward.initial))
        self.nodes_created = 1

        frontiers_backward = []

        # calculate backwards frontiers
        for problem in self.problems_backward:
            pq = PriorityQueue(self.get_a_star_heuristic(problem), self.g)
            frontiers_backward.append(pq)
            frontiers_backward[-1].append(Node(problem.initial))
            self.nodes_created += 1

        # only one of each expanded lists
        expanded_forward = {}
        expanded_backward = {}

        while frontier_forward and any(frontier for frontier in frontiers_backward):
            cheapest_backward_frontier = min(frontiers_backward, key=lambda frontier: self.get_f_value(frontier))

            # the combined f values of the priority node in the forward frontier and the cheapest backward frontier
            combined_f_values = self.get_f_value(frontier_forward) + self.get_f_value(cheapest_backward_frontier)

            if combined_f_values < self.lowest_cost_so_far:
                if self.top_node_has_min_f_value(frontier_forward, cheapest_backward_frontier):
                    self.step('forward', frontier_forward, cheapest_backward_frontier, expanded_forward, expanded_backward)
                else:
                    self.step('backward', cheapest_backward_frontier, frontier_forward, expanded_backward, expanded_forward)
            else:
                break

    def step(self, direction: str, frontier_a, frontier_b, expanded_a, expanded_b):
        """Expands a node and checks for a solution."""
        node = frontier_a.pop()
        state = node.state

        expanded_a[state] = node

        # solution found when expanded node is in the other direction's expanded list
        if state in expanded_b:
            # calculate new solution and lowest solution cost
            new_solution = self.make_solution(direction, node, expanded_b[state])
            self.lowest_cost_so_far = min(self.lowest_cost_so_far, frontier_a.f(node) + frontier_b.f(node))

            # if the new solution is cheaper, then it becomes the current solution
            if self.solution is None or len(new_solution) < len(self.solution):
                self.solution = new_solution

        # expand frontier
        # direction of problem doesn't matter when expanding node
        for child in node.expand(self.problem_forward):
            state = child.state
            if state not in expanded_a:
                frontier_a.append(child)
                self.nodes_created += 1

    def make_backward_problems(self, problem: 'RobotNavigationProblem') -> list:
        """Creates backward problems where the initial and goal states are reversed. For each different goal state,
        a new backward problem will be created."""
        problems_backward = []

        for goal_state in problem.goal:
            new_backward_problem = copy.deepcopy(problem)
            new_backward_problem.initial, new_backward_problem.goal = goal_state, [problem.initial]
            problems_backward.append(new_backward_problem)

        return problems_backward

    def get_f_value(self, frontier):
        """Returns the f value of the item with the highest priority in the priority queue."""
        if frontier:
            return frontier.f(frontier.peak())
        return numpy.inf

    def make_solution(self, direction: str, node_a: 'Node', node_b: 'Node') -> list:
        """The solution is a list of actions that combines node_a and node_b. The actions of the goal node are
        reversed, depending on the direction."""
        solution = []

        if direction == 'forward':
            actions_a = node_a.solution()
            actions_b = [utils.reverse_action(action) for action in node_b.solution()]
            actions_b.reverse()
            solution = actions_a + actions_b
            self.goal_node = node_b.path()[0]
        elif direction == 'backward':
            actions_a = [utils.reverse_action(action) for action in node_a.solution()]
            actions_a.reverse()
            actions_b = node_b.solution()
            solution = actions_b + actions_a
            self.goal_node = node_a.path()[0]
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

    def get_a_star_heuristic(self, problem: 'RobotNavigationProblem'):
        """Gets the ordering function for the priority queues of different problems."""
        return lambda node: node.path_cost + problem.heuristic(node.state)

    def print_solution(self):
        if self.is_solved():
            print(f"{self.goal_node} {self.nodes_created}")
            utils.print_solution(self.solution)
        else:
            print(f"No goal is reachable; {self.nodes_created}")
