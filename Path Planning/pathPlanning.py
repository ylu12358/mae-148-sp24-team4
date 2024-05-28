import itertools
import numpy as np
from scipy.spatial.distance import cdist
from functions import paramInterp, convertToWGS84

def pathPlanning(origin, pickup, dropoff, avoid, avoid_offset, cardyn):
    # Determine permutation of every drop off index
    drop_perms = np.array(list(itertools.permutations(range(0, len(dropoff)))))

    # Generate lines for avoidance algorithm and for plotting
    avoid_lines_plot = np.zeros((5, 2, len(avoid)))
    avoid_lines_main = np.zeros((5, 2, len(avoid)))

    for i in range(0, len(avoid)):
        # Store obstacle coordinates at this index
        x = avoid[i, 0]
        y = avoid[i, 1]
        w = avoid[i, 2]/2
        h = avoid[i, 3]/2
        wo = w + avoid_offset
        ho = h + avoid_offset

        bl = np.array([x - w, y - h])
        tl = np.array([x - w, y + h])
        tr = np.array([x + w, y + h])
        br = np.array([x + w, y - h])
        blo = np.array([x - wo, y - ho])
        tlo = np.array([x - wo, y + ho])
        tro = np.array([x + wo, y + ho])
        bro = np.array([x + wo, y - ho])

        avoid_lines_plot[:, :, i] = np.vstack((bl, tl, tr, br, bl))
        avoid_lines_main[:, :, i] = np.vstack((blo, tlo, tro, bro, blo))

    # Initialize shortest variable
    shortest = 1e6

    # Generate coordinate path by looping through every permutation of order and determining the path with minimum distance
    for p in range(0, len(drop_perms)):
        # Determine drop off order this iteration
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
            
            # Define local path
            local_x = local_temp[i+c, 0]
            local_y = local_temp[i+c, 1]
            local_x_before = local_temp[i+c-1, 0]
            local_y_before = local_temp[i+c-1, 1]

            # Determine direction of travel
            x_sign = local_x - local_x_before
            y_sign = local_y - local_y_before

            # Determine x and y function of current path
            def fx(x):
                result = (local_y - local_y_before) / (local_x - local_x_before) * (x - local_x_before) + local_y_before
                return result
            
            def fy(y):
                result = (local_x - local_x_before) / (local_y - local_y_before) * (y - local_y_before) + local_x_before

            # Sort the obstacles in order from closest to farthest on current path
            dist_dropoff_avoid = cdist(local_temp[i+c-1, :], avoid[:, 0:1])
            avoid_lines = np.zeros_like(avoid_lines_main)

            sorted_indices = np.argsort(dist_dropoff_avoid)
            dist_dropoff_avoid_main = dist_dropoff_avoid[sorted_indices]

            for m, i in enumerate(sorted_indices):
                avoid_lines[:, :, m] = avoid_lines_main[:, :, i]

            dist_current = cdist(local_temp[i+c-1, :], local_temp[i+c, :])

            # Determine intersection points for each obstacle
            for j in range
            ##### FINISH MEEEEEEEEE from line 130 to 325
        
        # Determine total distance of current GPS path
        dist_total = 0
        for q in range(1, len(local_temp)):
            dist_total += cdist(local_temp[q - 1, :], local_temp[q, :])
        
        # Check if new path is shorter than previous shortest path
        if shortest > dist_total:
            GPS_shortest = local_temp
            shortest = dist_total

    # # FINAL COLLISION CHECK
    # # Loop back through optimized path to ensure no collisions occur
    # flag = True
    # local_temp = GPS_shortest

    # # Loop until no collisions detected
    # count = 0
    # while flag:
    #     # Run at least twice to ensure no collisions are generated in the first path
    #     if count != 0:
    #         flag = False
    #     c = 0
    #     count += 1
    #     for i in 
        
        


    # Save optimized path
    local = local_temp

    # Parametric interpolation
    sp_sol = paramInterp(local, cardyn)

    # Convert coordinates to WGS84
    GPS = convertToWGS84(sp_sol)

    return GPS, sp_sol, local, avoid_lines_plot