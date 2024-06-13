import numpy as np
from pathPlanning import pathPlanning
from functions import plotEnvironment

# Define origin for local coordinates in WGS84 (GPS)
#                   LAT         LON
origin = np.array([32.881110, -117.235472])

#Define pickup and drop off in local coordinates (m)
#                  X  Y
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
GPS_coords, interp, path = pathPlanning(origin, pickup, dropoff, obstacle, offset, cardyn)

# Find first point
ref_point = GPS_coords[0, :2]

# Shift all points by subtracting the reference point
GPS_coords_normalized = GPS_coords.copy()
GPS_coords_normalized[:, 0] -= ref_point[0]
GPS_coords_normalized[:, 1] -= ref_point[1]

# Print to terminal
print(GPS_coords_normalized)

# Plot
fig = plotEnvironment(pickup, dropoff, obstacle, path, interp, GPS_coords_normalized)