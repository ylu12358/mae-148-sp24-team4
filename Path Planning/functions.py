import numpy as np


def checkCollision(obstacle, path):
    # Obstacle is a 4x2 array containing the XY coordinates of the obstacle
    # Path is a 2x2 array containing the XY coordinates of the start and end points of the path

    # Calculate and store X and Y signs
    x_sign = 1 if path[0,0] < path[1,0] else -1
    y_sign = 1 if path[0,1] < path[1,1] else -1

    # Unpack obstacle and path
    nw, ne, sw, se = XYWHToCorners(*obstacle)
    a1, a2 = path

    # Check for side collisions
    left = lineLineIntersect(a1, a2, nw, sw)
    bottom = lineLineIntersect(a1, a2, sw, se)
    right = lineLineIntersect(a1, a2, se, ne)
    top = lineLineIntersect(a1, a2, ne, nw)

    # Return intersection
    return left or bottom or right or top


def lineLineIntersect(a1, a2, b1, b2):
    # Unpack points
    a1x, a1y = a1
    a2x, a2y = a2
    b1x, b1y = b1
    b2x, b2y = b2

    # Calculate normalized distance ot intersection point
    uA = ( (b2x-b1x) * (a1y-b1y) - (b2y-b1y) * (a1x-b1x) ) / ( (b2y-b1y) * (a2x-a1x) - (b2x-b1x) * (a2y-a1y) )
    uB = ( (a2x-a1x) * (a1y-b1y) - (a2y-a1y) * (a1x-b1x) ) / ( (b2y-b1y) * (a2x-a1x) - (b2x-b1x) * (a2y-a1y) )

    # Return intersection
    return 0 <= uA <= 1 and 0 <= uB <= 1


def XYWHToCorners(x, y, w, h):
    nw = np.array([x - w/2, y + h/2])
    ne = np.array([x + w/2, y + h/2])
    sw = np.array([x - w/2, y - h/2])
    se = np.array([x + w/2, y - h/2])
    return nw, ne, sw, se


def plotEnvironment(pickup, dropoff, obstacle, local):
    # Import matplotlib
    import matplotlib.pyplot as plt
    
    # Establish variables for use in plotting
    p = np.vstack((pickup, dropoff))
    colors = np.random.rand(len(dropoff) + 1, 3)
    plt.figure(1)
    ax = plt.gca()

    # Plot all avoidance objects
    for i in range(obstacle.shape[0]):
        # Unpack corners of object
        nw, ne, sw, se = XYWHToCorners(*obstacle[i, :])

        # Combine corners into one variable for plotting
        corners = np.array([nw, ne, se, sw, nw])

        # Plot corners and fill rectangle
        plt.plot(corners[:, 0], corners[:, 1])
        plt.fill(corners[:, 0], corners[:, 1], alpha=0.5)

    # Plot the pickup and drop off locations
    for i in range(len(dropoff)):
        if i == 0:
            plt.plot(p[i, 0], p[i, 1], '^', color=colors[i], linewidth=2)
        else:
            plt.plot(p[i, 0], p[i, 1], 'o', color=colors[i], linewidth=2)

    # Plot the path lines
    plt.plot(local[:, 0], local[:, 1], 'k--')

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
    tstep = cardyn.dx / cardyn.vel_avg
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