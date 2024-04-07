import sys
import search_methods as sm
from problem import RobotNavigationProblem
from action import JumpAction
import utils


def select_search_method(method_name: str, problem: 'RobotNavigationProblem') -> 'SearchMethod':
    method_name = method_name.lower()

    if method_name == 'dfs':
        return sm.DepthFirstSearch(problem)
    elif method_name == 'bfs':
        return sm.BreadthFirstSearch(problem)
    elif method_name == 'gbfs':
        return sm.GreedyBestFirstSearch(problem)
    elif method_name in ['as', 'a*']:
        return sm.AStarSearch(problem)
    elif method_name == 'cus1':
        return sm.IterativeDeepeningSearch(problem)
    elif method_name == 'cus2':
        return sm.BidirectionalAStarSearch(problem)
    elif method_name == 'ucs':
        return sm.UniformCostSearch(problem)

    raise RuntimeError(f"There is no search method named {method_name.upper()}")


def main():
    if len(sys.argv) < 3:
        raise RuntimeError("Command line arguments are missing")

    filename = sys.argv[1]
    search_method_name = sys.argv[2]
    problem = RobotNavigationProblem(filename)
    print_grid = False

    if 'jump' in sys.argv:
        problem.action_strategy = JumpAction()
    if 'print_grid' in sys.argv:
        print_grid = True

    search_method = select_search_method(search_method_name, problem)
    search_method.solve()

    if print_grid:
        utils.print_grid(problem.grid)

    print(filename, search_method.method_name)
    search_method.print_solution()


if __name__ == '__main__':
    main()
