% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
addpath('A_star')

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 28.0;
yTarget = 23.0;
MAX_X = 30;
MAX_Y = 25;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);

% Waypoint Generator Using the A* 
greedy_h = 0.5;
greedy_g = 0.5;
%'greedy_h' for biasing on h(n), agent tend to go to target
%'greedt_g' for biasing on g(n), agent tend to explore
path = A_star_search(map, MAX_X,MAX_Y,greedy_h,greedy_g);

% visualize the 2D grid map
figure,
visualize_map(map, path, []);

% save map
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
