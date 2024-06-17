import itertools
import numpy as np
from scipy.spatial.distance import cdist
from functions import correctCollisions, paramInterp, convertToWGS84, convertToUTM, rotateOrigin

def pathPlanning(angle, origin, pickup, dropoff, obstacle, offset, cardyn):
    # Determine permutation of every drop off index
    drop_perms = np.array(list(itertools.permutations(range(0, len(dropoff)))))

    # Initialize shortest variable
    shortest = 1e6

    # Generate coordinate path by looping through every permutation of order and determining the path with minimum distance
    for p in range(len(drop_perms)):
        # Store drop off order this iteration
        path_index = drop_perms[p, :]

        # print(path_index)

        # Generate coordinate path
        path_temp = pickup
        c = 0

        for i in range(1, len(dropoff)+2):
            # Add current dropoff point to temporary path
            path_temp = np.vstack((path_temp, pickup)) if i == len(dropoff)+1 else np.vstack((path_temp, dropoff[path_index[i-1], :]))

        # Fix obstacle collisions
        # path_temp = correctCollisions(path_temp, obstacle, offset)

        # Determine total distance of current GPS path
        dist_total = 0
        for q in range(1, len(path_temp)):
            dist_total += cdist([path_temp[q - 1, :]], [path_temp[q, :]])
        
        # Check if new path is shorter than previous shortest path
        if shortest > dist_total:
            GPS_shortest = path_temp
            shortest = dist_total

    # Save optimized path
    path_local = GPS_shortest
    path_local = correctCollisions(path_local, obstacle, offset)

    # Parametric interpolation
    interp = paramInterp(path_local, cardyn)

    # Convert coordinates to WGS84
    # GPS = convertToWGS84(interp, origin)
    GPS_UTM = convertToUTM(interp, origin)

    # Find first point
    ref_point = GPS_UTM[0, :2]

    # Shift all points by subtracting the reference point
    GPS = GPS_UTM.copy()
    GPS[:, 0] -= ref_point[0]
    GPS[:, 1] -= ref_point[1]
    
    # Rotate points to match with real orientation
    GPS_rotated = rotateOrigin(GPS, angle)
    

    return GPS_rotated, interp, path_local
