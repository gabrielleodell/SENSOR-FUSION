#! usr/bin python
import random

import main
import math
import config
import operator

from collections import deque


# Set up nodes to keep track of parents
class Node:

    def __init__(self, position, parent, cost):
        self.position = position
        self.parent = parent
        self.cost = cost


obstacles = config.obstacles
rows = config.rows
columns = config.columns
start = Node(config.start, None, 0)
goal = Node(config.goal, None, 0)


# Returns the cost of the node
def get_cost(node):
    return node.cost


# Gets neighboring cells
def get_neighbors(position, dfs):
    neighbors = []
    r, c = position

    up = (r - 1, c)
    down = (r + 1, c)
    left = (r, c - 1)
    right = (r, c + 1)

    # Movement order is different in DFS - up, left, right, down
    if dfs:
        if check_valid(up): neighbors.append(up)
        if check_valid(left): neighbors.append(left)
        if check_valid(right): neighbors.append(right)
        if check_valid(down): neighbors.append(down)

    # Movement order - left, down, up, right
    else:
        if check_valid(left): neighbors.append(left)
        if check_valid(down): neighbors.append(down)
        if check_valid(up): neighbors.append(up)
        if check_valid(right): neighbors.append(right)

    return neighbors


# Check if cell is in bounds and not an obstacle
def check_valid(cell):
    if cell[0] < 0 or cell[1] < 0 or cell[0] > (rows - 1) or cell[1] > (columns - 1) or cell in obstacles:
        return False
    else:
        return True


# Returns the Manhattan distance from start
def distance_from_start(position):
    return abs(position[0] - start.position[0]) + abs(position[1] - start.position[1])


# Returns the Manhattan distance to goal
def distance_to_goal(position):
    return abs(position[0] - goal.position[0]) + abs(position[1] - goal.position[1])


# Returns the diagonal distance to goal
def diagonal_distance(position):
    c = math.sqrt(2)
    dx = abs(position[0] - goal.position[0])
    dy = abs(position[1] - goal.position[1])
    return (c * min(dy, dx)) + (max(dy, dx) - min(dy, dx))


# Returns the euclidean distance to goal
def euclidean_distance(position):
    return math.hypot((goal.position[0] - position[0]), (goal.position[1] - position[1]))


# Reverse engineer path from search and display
def find_path(current):
    path = []

    while current != start:
        path.append(current.position)
        current = current.parent

    path.append(start.position)
    path.reverse()
    return path


# Implementation of the breadth first search (BFS) algorithm
def breadth_first_search():
    counter = 0

    searched = deque(start.position)
    frontier = deque([start])

    # Keep going until no moves left or goal reached
    while frontier:
        counter = counter + 1

        # Check for exit request
        if main.pygame.event.peek(main.pygame.QUIT):
            main.pygame.quit()

        current = frontier.popleft()                # Pull next node from frontier
        main.display_search(current.position)       # Update graphics

        # Exit if goal reached
        if current.position == goal.position:
            print('Goal found')
            print('BFS complete')
            print('Iterations: %i' % counter)
            return find_path(current)

        neighbors = get_neighbors(current.position, False)

        # Add new neighbors to the frontier
        for n in neighbors:
            neighbor = Node(n, current, None)

            # Only add cells that have not been searched before
            if neighbor.position not in searched:
                searched.append(neighbor.position)
                frontier.append(neighbor)

    # Will only reach this if it fails
    print("Robot is stuck and cannot reach goal. Ending search.")
    return []


# Implementation of the depth first search (DFS) algorithm
def depth_first_search(randomized):
    counter = 0

    frontier = deque([(start, [start.position])])       # Node, path (positions only)
    searched = deque(start.position)

    # Keep going until no moves left or goal reached
    while frontier:
        counter = counter + 1

        # Check for exit request
        if main.pygame.event.peek(main.pygame.QUIT):
            main.pygame.quit()

        (current, path) = frontier.pop()                # LIFO
        main.display_search(current.position)           # Update graphics

        # Exit if goal reached
        if current.position == goal.position:
            print('Goal found')
            print('DFS complete')
            print('Iterations: %i' % counter)
            return path

        neighbors = get_neighbors(current.position, True)

        if randomized:
            random.shuffle(neighbors)

        # Add new neighbors to the frontier
        for n in neighbors:
            neighbor = Node(n, current, None)

            if neighbor.position not in searched:
                searched.append(neighbor.position)
                frontier.append((neighbor, path + [neighbor.position]))

    # Will only reach this if it fails
    print("Robot is stuck and cannot reach goal. Ending search.")
    return []


# Implementation of the Dijkstra search algorithm
def dijkstra_search():
    counter = 0
    move = 1

    frontier = [start]
    searched = []

    # Populate frontier with all possible nodes
    for r in range(rows):
        for c in range(columns):
            node = Node((r, c), None, float('inf'))

            if node not in obstacles:
                frontier.append(node)

    # Keep going until no moves left or goal reached
    while frontier:
        counter = counter + 1

        # Sort queue by cost (max -> min)
        frontier.sort(key=get_cost, reverse=True)
        current = frontier.pop()

        searched.append(current.position)
        main.display_search(current.position)

        # Check for exit request
        if main.pygame.event.peek(main.pygame.QUIT):
            main.pygame.quit()

        # If goal reached, display path and wait for user to close
        if current.position == goal.position:
            print('Goal found')
            print('Dijkstra search complete')
            print('Iterations: %i' % counter)
            return find_path(current)

        neighbors = get_neighbors(current.position, False)

        # Checks if neighbors are unknown and add to searched
        for n in neighbors:
            neighbor = Node(n, current, (current.cost + move))

            if neighbor.position not in searched:
                searched.append(neighbor.position)

                # Add it to frontier or compare to existing calculation, keep best cost
                if neighbor.position not in frontier:
                    frontier.append(neighbor)

                else:
                    queue_id = frontier.index(neighbor.position)

                    if frontier[queue_id].cost > neighbor.cost:
                        frontier[queue_id] = neighbor


# Implementation of the A* search algorithm
def a_star():
    counter = 0
    searched = []
    frontier = [start]

    # Keep going until no moves left or goal reached
    while frontier:
        counter = counter + 1

        frontier.sort(key=get_cost)
        current = frontier.pop()

        for i, node in enumerate(frontier):
            cost = euclidean_distance(node.position)

            if (node.cost + cost) < current.cost:
                current = node
                frontier.pop(i)

        searched.append(current.position)
        main.display_search(current.position)

        # Check for exit request
        if main.pygame.event.peek(main.pygame.QUIT):
            main.pygame.quit()

        # If goal reached, display path and wait for user to close
        if current.position == goal.position:
            print('Goal found')
            print('A* search complete')
            print('Iterations: %i' % counter)
            return find_path(current)

        neighbors = get_neighbors(current.position, False)

        # Checks if neighbors are unknown and add to queue
        for n in neighbors:
            heuristic = euclidean_distance(n)
            cost = current.cost + heuristic

            neighbor = Node(n, current, cost)

            if neighbor.position in searched:
                continue

            elif neighbor.position not in frontier:
                frontier.append(neighbor)
            else:
                i = frontier.index(neighbor.position)

                if frontier[i].cost > neighbor.cost:
                    frontier[i] = neighbor
