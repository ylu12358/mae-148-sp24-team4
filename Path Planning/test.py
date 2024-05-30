import numpy as np
# from pathPlanning import pathPlanning
# from functions import plotEnvironment
from functions import checkCollision, plotEnvironment

# Define origin for local coordinates in WGS84 (GPS)
#           LAT         LON         ALT
origin = np.array([32.881110, -117.235472, 10.0])

#Define pickup and drop off in local coordinates (m)
#       X  Y
pickup = np.array([5, 15])

dropoff = np.array([
    [ 3,  9],
    [18, 18],
    [18,  3],
    [ 5,  4],
    [ 2, 19],
    [ 8,  2],
    [10, 18]
])

#Define center of each obstacle in local coordinates and the width and height of each obstacle (m)
#     X   Y    W    H
obstacle = np.array([
    [ 3,  4, 1.5, 1.5], 
    [ 3, 12, 2.5,   2],
    [ 4,  1,   3, 1.5],
    [10,  2,   2, 1.5],
    [14, 16,   4,   4],
    [ 5, 18,   4,   1],
    [17,  9,   4,   5]
])

# Define offset to keep vehicle away from obstacles (m)
offset = 0.25

# Define car dynamics
#   Minimum turning radius (m)
#   Average velocity (m/s)
#   Distance step for GPS output (m)
cardyn = {
    'min_turn_r': 0.75,
    'vel_avg': 1,
    'dx': 0.2
}

# Run path planning function
# GPS_coords, sp_sol, local_coords, obst_coords = pathPlanning(origin, pickup, dropoff, avoid, offset, cardyn)

# Plot
# plotEnvironment(pickup, dropoff, obst_coords, local_coords)

# Display final coordinates
# print(GPS_coords)



# DELETE ME
path = np.array([[0,0], [1,3]])
print(checkCollision(obstacle[0, :], path))
plotEnvironment(pickup, dropoff, obstacle, path)