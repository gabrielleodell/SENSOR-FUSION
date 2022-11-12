import walls

visible = str(input('Would you like to see the full map? Y/N: '))
search = str(input('Enter search type (BFS, DFS, A*, Dijkstra): '))
sim = str(input('Output path for 3D simulation? Y/N: '))

visible = visible.lower()
search = search.lower()
sim = sim.lower()

randomized = False

if search == 'dfs':
    randomized = str(input('Randomize search? Y/N: '))

    if randomized.lower() == ('y' or 'yes'):
        randomized = True
    else:
        randomized = False

# Grid size
rows = 100
columns = 100

# Screen parameters
width = 8              # Change to 6 for not high-res
height = 8             # Change to 6 for not high-res
margin = 1

# Safety threshold for robot (expands obstacles)
threshold = 4

# Establish search parameters
start = (threshold, threshold)                          # Top left corner
goal = ((rows-threshold) - 1, (columns-threshold) - 1)      # Bottom right corner

# Generate the obstacles (based on user input)
obstacles = walls.generate_walls(threshold, rows, columns)

# Search status
running = True
