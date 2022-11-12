
def generate_walls(threshold, rows, columns):
    obstacles, safety = [], []
    obstacles.clear()
    safety.clear()

    # Assuming 100x100 grid size

    for i in range(0, 25):
        obstacles.append((i, 20))

    for i in range(50, 100):
        obstacles.append((10, i))

    for i in range(10, 30):
        obstacles.append((i, 75))

    for i in range(0, 50):
        obstacles.append((40, i))

    for i in range(60, 100):
        obstacles.append((i, 18))

    for i in range(18, 35):
        obstacles.append((90, i))

    for i in range(70, 100):
        obstacles.append((i, 60))

    for i in range(80, 100):
        obstacles.append((80, i))

    # Add safety threshold around obstacles
    for obs in obstacles:
        for r in range(-threshold, threshold + 1):
            for c in range(-threshold, threshold + 1):
                position = obs[0] + r, obs[1] + c
                safety.append(position)

    # If position not already in obstacles, add it
    for cell in safety:
        if cell not in obstacles:
            obstacles.append(cell)

    # Left border
    for i in range(0, rows):
        for t in range(threshold):
            obstacles.append((i, t))

    # Top border
    for i in range(0, columns):
        for t in range(threshold):
            obstacles.append((t, i))

    # Bottom border
    for i in range(rows-threshold, rows):
        for c in range(columns):
            obstacles.append((i, c))

    # Right border
    for i in range(columns-threshold, columns):
        for r in range(rows):
            obstacles.append((r, i))

    return obstacles
