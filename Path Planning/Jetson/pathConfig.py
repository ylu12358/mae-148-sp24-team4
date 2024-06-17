import numpy as np

def angle():
        # Define the clockwise heading (angle in degrees) from true north
        theta = 170
        
        return theta

def origin():

        # Define origin for local coordinates in WGS84 (GPS)
        #                   LAT         LON
        origin = np.array([32.88130222, -117.23540056])

        return origin



def dropoff(pathtype):

        # Define the dropoff locations in local coordinates (m)
        #         X   Y
        dropoff = [];
        if pathtype == 'Circle':
                dropoff = np.array([
                        [ 8.23,  12.37]
                ])
        elif pathtype == 'Square':
                dropoff = np.array([
                        [ 16.75,  5.73]
                ])

        else:
                dropoff = np.array([
                        [ 8.23,  12.37],
                        [ 16.75,  5.73],
                        [ 0.0, 3.0]
                ])

        return dropoff



def obstacle():

        #Define center of each obstacle in local coordinates and the width and height of each obstacle (m)
        #         X   Y    W    H
        obstacle = np.array([
                [ 3.15,  6.53, 1.52, 1.60],
                [ 10.78, 3.81, 2.22,   1.42]
        ])

        return obstacle



def offset():

        # Define the desired offset from obstacles vertices (m)
        offset = 0.25

        return offset



def cardyn():

        # Define the associated dynamics of your car
        cardyn = {
                'min_turn_r': 0.75,
                'vel_avg': 1,
                'dx': 0.2
        }

        return cardyn
