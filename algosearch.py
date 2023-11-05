from pq import PriorityQueue
from enum import Enum


class GhostChaseAlgorithm(Enum):
    BFS = "BFS"
    ASTAR = "ASTAR"


def reconstruct_path(came_from, start, end):
    reverse_path = [end]
    while end != start:
        end = came_from[end]
        reverse_path.append(end)
    return list(reversed(reverse_path))


def get_adjacent_cells(grid, cell):
    i, j = cell
    cells_list = []
    if 0 <= i + 1 < len(grid) and grid[i + 1][j] == 0:
        cells_list.append((i + 1, j))
    if 0 <= i - 1 < len(grid) and grid[i - 1][j] == 0:
        cells_list.append((i - 1, j))
    if 0 <= j + 1 < len(grid[0]) and grid[i][j + 1] == 0:
        cells_list.append((i, j + 1))
    if 0 <= j - 1 < len(grid[0]) and grid[i][j - 1] == 0:
        cells_list.append((i, j - 1))
    return cells_list


def get_manhattan_distance(cell, goal):
    (i, j) = cell
    return abs(goal[0] - i) + abs(goal[1] - j)


def a_star_search(start, goal, grid):
    visited = set()
    came_from = dict()
    distance = {start: 0}
    priority_queue = PriorityQueue()
    priority_queue.add(start)
    while len(priority_queue) > 0:
        node = priority_queue.pop()
        if node in visited:
            continue
        if node == goal:
            return reconstruct_path(came_from, start, node)
        visited.add(node)
        for child in get_adjacent_cells(grid, node):
            priority_queue.add(
                child,
                distance[node] + 1 + get_manhattan_distance(child, goal)
            )
            if (child not in distance
                    or distance[node] + 1 < distance[child]):
                distance[child] = distance[node] + 1
                came_from[child] = node
    return None


def bfs_search(start, goal, grid):
    visited = set()
    came_from = dict()
    distance = {start: 0}
    priority_queue = PriorityQueue()
    priority_queue.add(start)
    while len(priority_queue) > 0:
        node = priority_queue.pop()
        if node in visited:
            continue
        if node == goal:
            return reconstruct_path(came_from, start, node)
        visited.add(node)
        for child in get_adjacent_cells(grid, node):
            if child not in distance:
                priority_queue.add(child, distance[node] + 1)
                distance[child] = distance[node] + 1
                came_from[child] = node
    return None


def shortest_path_binary_matrix(grid, start, goal, chase_algorithm):
    if chase_algorithm == GhostChaseAlgorithm.ASTAR:
        shortest_path = a_star_search(start, goal, grid)
    else:
        shortest_path = bfs_search(start, goal, grid)

    if shortest_path is None:
        return []
    else:
        return shortest_path
