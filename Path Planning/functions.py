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

            # print(obstacle_current)

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


def plotEnvironment(pickup, dropoff, obstacle, path, interp, GPS_coords):
    # Import matplotlib
    import matplotlib.pyplot as plt
    
    # Establish variables for use in plotting
    colors = np.random.rand(len(dropoff) + 1, 3)
    fig = plt.figure(1)
    ax1 = fig.add_subplot(121)

    # Plot all avoidance objects
    for i in range(len(obstacle)):
        # Unpack corners of object
        nw, ne, sw, se = convertXYWHtoCorners(*obstacle[i, :])

        # Combine corners into one variable for plotting
        corners = np.array([nw, ne, se, sw, nw])

        # Plot corners and fill rectangle
        ax1.plot(corners[:, 0], corners[:, 1])
        ax1.fill(corners[:, 0], corners[:, 1], alpha=0.5)

    # Plot the pickup and drop off locations
    ax1.plot(pickup[0], pickup[1], '^', color=colors[0], linewidth=2)
    for i in range(len(dropoff)):
        ax1.plot(dropoff[i, 0], dropoff[i, 1], 'o', color=colors[i+1], linewidth=2)

    # Plot the path lines
    ax1.plot(path[:, 0], path[:, 1], 'k--')

    # Plot the spline
    ax1.plot(interp[0], interp[1], 'r')

    # Graph formatting
    ax1.set_aspect('equal', adjustable='box')
    ax1.grid(True)
    ax1.set_title('Obstacles and Path')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    
    # Establish variables for use in plotting
    ax2 = fig.add_subplot(122)
    ax2.plot(GPS_coords[:, 0], GPS_coords[:, 1])

    # Graph formatting
    ax2.set_aspect('equal', adjustable='box')
    ax2.grid(True)
    ax2.set_title('UTM Converted, Normalized Points')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    plt.show()


def paramInterp(path, cardyn):
    # Import cubic spline library
    from scipy.interpolate import CubicSpline

    # Separate x and y coordinates
    x = path[:, 0]
    y = path[:, 1]

    # Define t parameter
    t = np.arange(len(x))

    # Create cubic spline interpolation functions
    sp_x = CubicSpline(t, x)
    sp_y = CubicSpline(t, y)

    # Generate a smooth set of points
    dt = cardyn['dx'] / cardyn['vel_avg']
    t_fine = np.arange(0, len(x) - 1 + dt, dt)
    x_smooth = sp_x(t_fine)
    y_smooth = sp_y(t_fine)

    # Combine into output variable
    interp = np.vstack((x_smooth, y_smooth))

    return interp


def convertToWGS84(input_coords, origin):
    # Import conversion library
    from pyproj import Proj, Transformer
    import numpy as np

    # WGS84 coordinates of the local origin (latitude and longitude in degrees)
    origin_lat = origin[0]
    origin_lon = origin[1]

    # Local coordinates relative to the origin (in meters)
    local_x = input_coords[0]
    local_y = input_coords[1]

    # Define the WGS84 coordinate system
    wgs84_proj = Proj(proj='latlong', datum='WGS84')

    # Define a local coordinate system with the origin at the WGS84 point
    local_proj = Proj(proj="aeqd", datum='WGS84', lat_0=origin_lat, lon_0=origin_lon)

    # Create a transformer to convert from the local coordinate system to WGS84
    transformer = Transformer.from_proj(local_proj, wgs84_proj, always_xy=True)

    # Convert local coordinates to WGS84
    lon, lat = transformer.transform(local_x, local_y)

    alt = np.zeros_like(lon)  # Assuming altitude is zero for all points

    # Combine the results into GPS array
    GPS = np.vstack((lat, lon, alt)).T

    return GPS


def convertToUTM(input_coords, origin):
    # Import conversion library
    from pyproj import Proj, Transformer
    import numpy as np

    # WGS84 coordinates of the local origin (latitude and longitude in degrees)
    origin_lat = origin[0]
    origin_lon = origin[1]

    # Local coordinates relative to the origin (in meters)
    local_x = input_coords[0]
    local_y = input_coords[1]

    # Define the WGS84 coordinate system
    wgs84_proj = Proj(proj='latlong', datum='WGS84')

    # Define the UTM coordinate system based on the origin's latitude and longitude
    utm_proj = Proj(proj="utm", zone=int((origin_lon + 180) / 6) + 1, datum='WGS84')

    # Define a local coordinate system with the origin at the WGS84 point
    local_proj = Proj(proj="aeqd", datum='WGS84', lat_0=origin_lat, lon_0=origin_lon)

    # Create a transformer to convert from the local coordinate system to UTM
    transformer = Transformer.from_proj(local_proj, utm_proj, always_xy=True)

    # Convert local coordinates to UTM
    easting, northing = transformer.transform(local_x, local_y)

    alt = np.zeros_like(easting)  # Assuming altitude is zero for all points

    # Combine the results into UTM array
    UTM = np.vstack((easting, northing, alt)).T

    return UTM