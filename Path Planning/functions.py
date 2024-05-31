import numpy as np
from geometry import checkLineRect, convertXYWHtoCorners, offsetRectangle


def correctCollisions(path, obstacle, offset):
    obstacle_copy = np.copy(obstacle)
    obstacle_offset = np.array([offsetRectangle(obstacle_current, offset) for obstacle_current in obstacle_copy])
    return correctCollisionsRecursive(path, obstacle_offset)


def correctCollisionsRecursive(path, obstacle):
    # # Base case
    # if isValidPath(path, obstacle):
    #     return path

    # Loop through every path segment
    for i in range(len(path)-1):
        # Extract path for this iteration
        path_current = np.array([path[i], path[i + 1]])

        # Loop through every obstacle
        for j in range(len(obstacle)):
            # Extract obstacle for this iteration
            obstacle_current = obstacle[j]

            print(obstacle_current)

            # Check if current path intersects with current obstacle
            intersects, reroute_corner = checkLineRect(obstacle_current, path_current)

            # If an intersection is found, reroute the path
            if intersects:
                # Create a new path that goes around the obstacle
                new_path = np.concatenate([path[:i + 1], [reroute_corner], path[i + 1:]])
                
                # Recursively find a valid path with the new path
                result_path = correctCollisionsRecursive(new_path, obstacle)
                
                if result_path is not None:
                    return result_path

    # Return path if no collisions found    
    return path


# def isValidPath(path, obstacle):
#     # Check if the path intersects with any obstacles
#     for i in range(len(path) - 1):
#         path_current = np.array([path[i], path[i + 1]])
#         for obstacle_current in obstacle:
#             intersects, _ = checkLineRect(obstacle_current, path_current)
#             if intersects:
#                 return False
#     return True


def plotEnvironment(pickup, dropoff, obstacle, path, interp):
    # Import matplotlib
    import matplotlib.pyplot as plt
    
    # Establish variables for use in plotting
    colors = np.random.rand(len(dropoff) + 1, 3)
    plt.figure(1)
    ax = plt.gca()

    # Plot all avoidance objects
    for i in range(len(obstacle)):
        # Unpack corners of object
        nw, ne, sw, se = convertXYWHtoCorners(*obstacle[i, :])

        # Combine corners into one variable for plotting
        corners = np.array([nw, ne, se, sw, nw])

        # Plot corners and fill rectangle
        plt.plot(corners[:, 0], corners[:, 1])
        plt.fill(corners[:, 0], corners[:, 1], alpha=0.5)

    # Plot the pickup and drop off locations
    plt.plot(pickup[0], pickup[1], '^', color=colors[0], linewidth=2)
    for i in range(len(dropoff)):
        plt.plot(dropoff[i, 0], dropoff[i, 1], 'o', color=colors[i+1], linewidth=2)

    # Plot the path lines
    plt.plot(path[:, 0], path[:, 1], 'k--')

    # Plot the spline
    plt.plot(interp[0], interp[1])

    # Graph formatting
    ax.set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


def paramInterp(local, cardyn):
    # Import cubic spline library
    from scipy.interpolate import CubicSpline

    # Create the cubic spline
    sp = CubicSpline(np.arange(local.shape[1]), local.T)

    # Define time steps
    tstep = cardyn['dx'] / cardyn['vel_avg']
    t = np.arange(0, sp.x[-1] + tstep, tstep)

    # Evaluate the spline at the given time steps
    sp_sol = sp(t).T

    return sp_sol


def convertToWGS84(input_coords):
    # Import conversion library
    from pyproj import Transformer

    # Transpose to get local
    local = input_coords.T

    # Create a zero array for z
    z = np.zeros((local.shape[1],))

    # Define the local coordinate system and WGS84 coordinate system
    local_crs = 'epsg:32611'  # UTM Zone 11N
    wgs84_crs = 'epsg:4326'  # WGS84

    # Create a transformer object
    transformer = Transformer.from_crs(local_crs, wgs84_crs, always_xy=True)

    # Perform the transformation
    lat, lon = transformer.transform(local[0], local[1])
    alt = z  # Assuming altitude is zero for all points

    # Combine the results into GPS array
    GPS = np.vstack((lat, lon, alt)).T

    return GPS