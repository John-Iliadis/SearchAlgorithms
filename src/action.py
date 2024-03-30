from dataclasses import dataclass
from enums import Direction, GridElement
from vector import Vector2i
from typing import List
from grid import Grid


@dataclass(unsafe_hash=True, order=True)
class Action:
    direction: 'Direction'
    magnitude: int


class ActionStrategy:
    def evaluate_actions(self, state: 'Vector2i', grid: 'Grid') -> List['Action']:
        raise NotImplementedError

    def print_actions(self, actions: List['Action']):
        raise NotImplementedError


class SingleStepAction(ActionStrategy):
    def evaluate_actions(self, state: 'Vector2i', grid: 'Grid') -> List['Action']:
        available_actions = []

        if state.y > 0 and grid.data[state.x][state.y - 1] != GridElement.WALL.value:
            available_actions.append(Action(Direction.up, 1))
        if state.x > 0 and grid.data[state.x - 1][state.y] != GridElement.WALL.value:
            available_actions.append(Action(Direction.left, 1))
        if state.y < grid.height - 1 and grid.data[state.x][state.y + 1] != GridElement.WALL.value:
            available_actions.append(Action(Direction.down, 1))
        if state.x < grid.width - 1 and grid.data[state.x + 1][state.y] != GridElement.WALL.value:
            available_actions.append(Action(Direction.right, 1))

        return available_actions

    def print_actions(self, actions: List['Action']):
        print('[', end='')
        for i in range(len(actions)):
            if i < len(actions) - 1:
                print(f"'{actions[i].direction.name}', ", end='', sep='')
            else:
                print(f"'{actions[i].direction.name}'", end='', sep='')
        print(']')


class JumpAction(ActionStrategy):
    def evaluate_actions(self, state: 'Vector2i', grid: 'Grid') -> List['Action']:
        available_actions = []

        # up
        i = state.y - 1
        while i >= 0:
            if grid.data[state.x][i] != GridElement.WALL.value:
                available_actions.append(Action(Direction.up, state.y - i))
                break
            i -= 1

        # left
        i = state.x - 1
        while i >= 0:
            if grid.data[i][state.y] != GridElement.WALL.value:
                available_actions.append(Action(Direction.left, state.x - i))
                break
            i -= 1

        # down
        i = state.y + 1
        while i < grid.height:
            if grid.data[state.x][i] != GridElement.WALL.value:
                available_actions.append(Action(Direction.down, i - state.y))
                break
            i += 1

        # right
        i = state.x + 1
        while i < grid.width:
            if grid.data[i][state.y] != GridElement.WALL.value:
                available_actions.append(Action(Direction.right, i - state.x))
                break
            i += 1

        return available_actions

    def print_actions(self, actions: List['Action']):
        print('[', end='')
        for i in range(len(actions)):
            if i < len(actions) - 1:
                print(f"'{actions[i].direction.name} ({actions[i].magnitude})', ", end='', sep='')
            else:
                print(f"'{actions[i].direction.name} ({actions[i].magnitude})'", end='', sep='')
        print(']')
