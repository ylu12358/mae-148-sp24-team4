clc;
clear;
close all;

x = 3;
y = 12;
w = 2;
h = 2;

a1 = [5 14];
a2 = [1 10];
ax = [a1(1) a2(1)];
ay = [a1(2) a2(2)];

nw = [x-w/2 y+h/2];
ne = [x+w/2 y+h/2];
sw = [x-w/2 y-h/2];
se = [x+w/2 y-h/2];
obstacle = [nw; ne; se; sw; nw]';

plot(obstacle(1,:), obstacle(2,:));
hold on;
plot(ax, ay);
axis equal;
grid on;