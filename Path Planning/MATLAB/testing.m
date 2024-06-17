clc;
clear;
close all;

figure(1);
hold on;
axis equal;
grid on;

pickup = [5, 15];

dropoff = [3,  9;
          18, 18; 
          18,  3;
           5,  4;
           2, 19;
           8,  2;
          10, 18];

avoid = [3,   4,   1.5,   1.5; 
         3,   12,  2.5,   2; 
         4,   1,     3,   1.5;
        10,   2,     2,   1.5;
        14,  16,     4,   4;
         5,  18,     4,   1;
        17,   9,     4,   5];

order = [0 1 2 3 5 6 4];

for i = 1:length(avoid)
    x = avoid(i,1);
    y = avoid(i,2);
    w = avoid(i,3);
    h = avoid(i,4);
    plotObstacle(x,y,w,h);
end
plotPath(pickup(1), pickup(2), dropoff(order(1)+1,1), dropoff(order(1)+1,2));
for i = 1:(length(order)-2)
    a1x = dropoff(order(i)+1,1);
    a1y = dropoff(order(i)+1,2);
    a2x = dropoff(order(i+1)+1,1);
    a2y = dropoff(order(i+1)+1,2);
    plotPath(a1x, a1y, a2x, a2y);
end

function plotObstacle(x, y, w, h)
    nw = [x-w/2 y+h/2];
    ne = [x+w/2 y+h/2];
    sw = [x-w/2 y-h/2];
    se = [x+w/2 y-h/2];
    obstacle = [nw; ne; se; sw; nw]';
    
    plot(obstacle(1,:), obstacle(2,:));
end

function plotPath(a1x, a1y, a2x, a2y)
    ax = [a1x a2x];
    ay = [a1y a2y];

    plot(ax, ay);
end