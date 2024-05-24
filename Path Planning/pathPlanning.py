import itertools
import numpy as np

def pathPlanning(origin, pickup, dropoff, avoid, offset, cardyn):
    # Determine permutation of every drop off index
    drop_perms = np.array(list(itertools.permutations(range(0, len(dropoff)))))

    # Generate lines for avoidance algorithm
    avoid_lines_plot = np.zeros((5, 2, max(avoid.shape)))
    avoid_lines_main = np.zeros((5, 2, max(avoid.shape)))

    for i in range(0, max(avoid.shape)):
        # Store obstacle coordinates at this index
        x = avoid[i, 0]
        y = avoid[i, 1]
        w = avoid[i, 2]/2
        h = avoid[i, 3]/2
        wo = w + offset
        ho = h + offset

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
    for p in range(0, max(drop_perms.shape)):
        # Determine drop off order this iteration
        path_index = drop_perms[p, :]

        # Generate coordinate path
        local_temp = pickup
        c = 0

        for i in range(1, max(dropoff.shape)+1):
            # Endpoints
            if i == max(dropoff.shape)+1:
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
            #dist_dropoff_avoid = line 114 on MATLAB

    GPS = 5
    return GPS