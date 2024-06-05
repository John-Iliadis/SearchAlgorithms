from action import JumpAction
from search_methods import *
import utils


def select_search_method(method_name: str, problem: 'RobotNavigationProblem') -> 'SearchMethod':
    method_name = method_name.lower()

    if method_name == 'dfs':
        return DepthFirstSearch(problem)
    elif method_name == 'bfs':
        return BreadthFirstSearch(problem)
    elif method_name == 'gbfs':
        return GreedyBestFirstSearch(problem)
    elif method_name in ['as', 'a*']:
        return AStarSearch(problem)
    elif method_name == 'cus1':
        return IterativeDeepeningSearch(problem)
    elif method_name == 'cus2':
        return BidirectionalAStarSearch(problem)
    elif method_name == 'ucs':
        return UniformCostSearch(problem)

    raise RuntimeError(f"There is no search method named {method_name.upper()}")


def main():
    if len(sys.argv) < 3:
        raise RuntimeError("Command line arguments are missing")

    filename = sys.argv[1]
    search_method_name = sys.argv[2]
    problem = RobotNavigationProblem(filename)
    print_grid = False

    if len(sys.argv) > 4:
        raise RuntimeError("Invalid number of command line arguments")
    elif len(sys.argv) == 4:
        if sys.argv[3].lower() == 'jump':
            problem.action_strategy = JumpAction()
        else:
            raise RuntimeError(f"{sys.argv[3]} is an invalid argument")

    search_method: 'SearchMethod' = select_search_method(search_method_name, problem)
    search_method.solve()

    if print_grid:
        utils.print_grid(problem.grid)

    print(filename, search_method.method_name)
    search_method.print_solution()


if __name__ == '__main__':
    main()
