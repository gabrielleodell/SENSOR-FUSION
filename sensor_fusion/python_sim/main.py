#! usr/bin python

import sys
import time
import pygame
import config
import tracemalloc

rows = config.rows
columns = config.columns
width = config.width
height = config.height
margin = config.margin
goal = config.goal
obstacles = config.obstacles
search = config.search
randomized = config.randomized
visible = config.visible

# Screen parameters
size = (rows * (height + margin)), (columns * (width + margin))
screen = pygame.display.set_mode(size)

# Define colors
black = (0, 0, 0)
white = (255, 255, 255)
gray = (160, 160, 160)
blue = (0, 0, 255)
navy = (2, 15, 115)
red = (255, 0, 0)
green = (0, 255, 0)


# Initial display
def display():

    # Calculate point robot location, if doing longer search stay at home until goal found
    center = ((width/2 + margin), (height/2 + margin))

    # Draw the grid
    for row in range(rows):
        for col in range(columns):

            # Give each shape different color
            if (row, col) in obstacles:
                color = black
            elif (row, col) == goal:
                color = green
            else:
                color = white

            pygame.draw.rect(screen, color, [(margin + width) * col + margin,
                            (margin + height) * row + margin, width, height])

            #pygame.draw.circle(screen, blue, center, width / 2)

        # Update screen with changes
        pygame.display.flip()


def display_search(cell):

    if cell == goal:
        color = green
    else:
        color = gray

    # Draw the grid
    pygame.draw.rect(screen, color, [(margin + width) * cell[1] + margin,
                    (margin + height) * cell[0] + margin, width, height])

    # Robot is stationary during searches
    center = (width / 2 + margin, height / 2 + margin)
    #pygame.draw.circle(screen, blue, center, width / 2)

    # Update screen with changes
    pygame.display.flip()


def display_robot(path):
    center = (width/2 + margin, height/2 + margin)

    for i in range(len(path)):

        # Check for exit request
        if pygame.event.peek(pygame.QUIT):
            pygame.quit()

        cell = path.pop(0)

        center = ((cell[1] * (width + margin) + (width/2 + margin)),
                  (cell[0] * (height + margin) + (height/2 + margin)))

        pygame.draw.circle(screen, blue, center, width / 2)
        pygame.display.flip()
        time.sleep(0.25)

        # Clear cell after
        pygame.draw.rect(screen, red, [(margin + width) * cell[1] + margin,
                        (margin + height) * cell[0] + margin, width, height])

    pygame.draw.circle(screen, blue, center, width/2)


def display_path(path):

    for cell in path:
        # Draw the grid
        pygame.draw.rect(screen, red, [(margin + width) * cell[1] + margin,
                        (margin + height) * cell[0] + margin, width, height])

    # Update screen with changes
    pygame.display.flip()


def display_waypoints(waypoints):

    for cell in waypoints:
        pygame.draw.rect(screen, blue, [(margin + width) * cell[1] + margin,
                        (margin + height) * cell[0] + margin, width, height])

    # Update screen with changes
    pygame.display.flip()


def convert_to_meters(path):
    meters = []

    for cell in path:
        x = (cell[0] * (height + margin) + (height / 2 + margin)) / 100
        y = (cell[1] * (width + margin) + (width / 2 + margin)) / 100

        meters.append((x, y))

    return meters


# Close screen when requested
def close_screen():
    while config.running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                config.running = False

    pygame.quit()


if __name__ == "__main__":
    import algorithms
    from algorithms import Node

    sys.setrecursionlimit(10**6)    # Extend limit for DFS

    # Initialize screen
    pygame.init()
    screen.fill(black)

    if visible == ('y' or 'yes'):
        display()
        time.sleep(0.5)

    #pause = str(input('Press enter to start search'))

    tracemalloc.start()
    t = time.time()

    # Initiate search algorithm (based on user input)
    if search == 'bfs':
        pygame.display.set_caption('Breadth First Search')
        result = algorithms.breadth_first_search()

    elif search == 'dfs':
        pygame.display.set_caption('Depth First Search')
        result = algorithms.depth_first_search(randomized)

    elif search == ('a*' or 'astar'):
        pygame.display.set_caption('A* Search')
        result = algorithms.a_star()

    else:
        pygame.display.set_caption("Dijkstra's Search")
        result = algorithms.dijkstra_search()

    if result:
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()

        print('Total time: ' + str(time.time() - t))
        print('Memory: ' + str(peak / 10**6))
        print('Path length: %i' % len(result))

        display_path(result)

        waypoints = result[::27]
        waypoints.append(goal)
        display_waypoints(waypoints)

        # If running the 3D simulation after, output path to a text file
        if config.sim.lower() == ('y' or 'yes'):
            path = convert_to_meters(waypoints)
            path.insert(0, (0.0, 0.0))

            # Write the position to the file in the format of "X, Y"
            with open("path.txt", "w") as file:
                for waypoint in path:
                    line = str(waypoint[1]) + ',' + str(waypoint[0])
                    file.write(line)
                    file.write('\n')
            file.close()
    else:
        print("Search failed. Please try again.")

    close_screen()
