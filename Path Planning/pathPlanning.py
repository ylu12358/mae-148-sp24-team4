import itertools
import numpy as np
from scipy.spatial.distance import cdist
from functions import paramInterp, convertToWGS84

def pathPlanning(origin, pickup, dropoff, obstacle, avoid_offset, cardyn):
    # Determine permutation of every drop off index
    drop_perms = np.array(list(itertools.permutations(range(0, len(dropoff)))))

    # Generate lines for avoidance algorithm and for plotting
    avoid_lines = np.zeros((5, 2, len(obstacle)))

    for i in range(len(obstacle)):
        # Store obstacle coordinates at this index
        x = obstacle[i, 0]
        y = obstacle[i, 1]
        w = obstacle[i, 2]/2
        h = obstacle[i, 3]/2
        wo = w + avoid_offset
        ho = h + avoid_offset

        blo = np.array([x - wo, y - ho])
        tlo = np.array([x - wo, y + ho])
        tro = np.array([x + wo, y + ho])
        bro = np.array([x + wo, y - ho])

        avoid_lines[:, :, i] = np.vstack((blo, tlo, tro, bro, blo))

    # Initialize shortest variable
    shortest = 1e6

    # Generate coordinate path by looping through every permutation of order and determining the path with minimum distance
    for p in range(len(drop_perms)):
        # Store drop off order this iteration
        path_index = drop_perms[p, :]

        # Generate coordinate path
        local_temp = pickup
        c = 0

        for i in range(1, len(dropoff)+1):
            # Endpoints
            if i == len(dropoff)+1:
                local_temp = np.vstack((local_temp, pickup))
            else:
                local_temp = np.vstack((local_temp, dropoff[path_index[i-1], :]))
        
        # Determine total distance of current GPS path
        dist_total = 0
        for q in range(1, len(local_temp)):
            dist_total += cdist([local_temp[q - 1, :]], [local_temp[q, :]])
        
        # Check if new path is shorter than previous shortest path
        if shortest > dist_total:
            GPS_shortest = local_temp
            shortest = dist_total

    # Save optimized path
    local = GPS_shortest

    # Parametric interpolation
    sp_sol = paramInterp(local, cardyn)

    # Convert coordinates to WGS84
    GPS = convertToWGS84(sp_sol)

    return GPS, sp_sol, local