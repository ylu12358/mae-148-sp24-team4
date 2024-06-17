clear
clc
close all

tic
%--------------------------------------------------------------------------%
%                                  INPUTS                                  %
%--------------------------------------------------------------------------%

% Define the origin that was used as (0, 0) for local cooridnates in WGS84
% coordinates (standard GPS)
%            LAT         LON       ALT
origin = [32.881110, -117.235472, 10.0];

% Define the pickup in local cooridnates (meters)
%         X   Y
pickup = [5, 15];

% Define each drop off on local coordinates (meters)
%          X   Y
dropoff = [3,  9;
          18, 18; 
          18,  3;
           5,  4;
           2, 19;
           8,  2;
          10, 18];

% Define the center of each obstacle in local coordinates and the width and
% height of each obstacle (meters)
%        X    Y     W      H
avoid = [3,   4,   1.5,   1.5; 
         3,   12,  2.5,   2; 
         4,   1,     3,   1.5;
        10,   2,     2,   1.5;
        14,  16,     4,   4;
         5,  18,     4,   1;
        17,   9,     4,   5];

% Define offset to keep vehicle away from obstacles (meters)
offset = 0.25;

% Define the car dynamics min turning radius (meters)
cardyn.mtr = 0.75;

% Define the car dynamics average velocity (meters per second)
cardyn.vave = 1;

% Define the car dynamics distance step for GPS output (meters)
cardyn.dstep = 0.2;

% Call the pathPlanning function to optimize the path
%GPS = pathPlanning(origin, pickup, dropoff, avoid, offset, cardyn)

% Plot the track in the WGS84 frame
%zoomLevel = 20;
%player = geoplayer(GPS(1, 1), GPS(1, 2), zoomLevel);
%plotRoute(player, GPS(:, 1), GPS(:, 2));


ang = 285;


x = (0:0.1:10)';

GPS = [x, x.^2];

rot = [cosd(ang), sind(ang); -sind(ang), cosd(ang)];


GPS_rot = rot*GPS';

GPS_rot = GPS_rot';

figure(1)
plot(GPS(:, 1), GPS(:, 2), 'r', GPS_rot(:, 1), GPS_rot(:, 2), 'b')
legend('Original', 'Rotated', 'Location', 'best')


toc