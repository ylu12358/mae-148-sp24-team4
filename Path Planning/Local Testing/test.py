import numpy as np
from pathPlanning import pathPlanning
from functions import plotEnvironment, rotateOrigin

# Define origin for local coordinates in WGS84 (GPS)
#                   LAT         LON
origin = np.array([32.881110, -117.235472])

# Define CW angle from north to shift the frame (degrees)
heading = 170

#Define pickup and drop off in local coordinates (m)
#                  X  Y
pickup = np.array([0, 0])

dropoff = np.array([
    [0, 3],
    [ 8.23, 12.37],
    [ 16.75,  5.73]
])

#Define center of each obstacle in local coordinates and the width and height of each obstacle (m)
#     X   Y    W    H
obstacle = np.array([
    [ 3.15,  6.53, 1.52, 1.60], 
    [10.78,  3.81, 2.22,  1.42]
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

# Rotates all points using function
GPS_coords_norm_rot = rotateOrigin(GPS_coords, heading)

# Convert the coords to csv and stor in desired path
file_path = "./sortbotpath.csv"
np.savetxt(file_path, GPS_coords_norm_rot, delimiter=",")

# Print to terminal
print(GPS_coords)
print(len(GPS_coords))

# Plot
fig = plotEnvironment(pickup, dropoff, obstacle, path, interp, GPS_coords, GPS_coords_norm_rot)