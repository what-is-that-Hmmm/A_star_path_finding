# A_star_path_finding
A repo for controlling and backup the versions of MATLAB file.
It is completed based on SHENLAN's assignment.

Author: Ziyue Tong

The g(n) is calculated by taking the lower integer of the Euclidean distance.
The h(n) is the heuristic cost, which is the exact calculated value of Euclidian distance. 

-- Setting the path generation policy: Normally, for simple map, i.e. only 
few obstacles in the environment, greedy_h>greedy_g to ensure planning speed.
For senarios like buildings, mazes, greedy_g>greedy_h to avoid local optimal.
greedy_h = 0.6;
greedy_g = 0.4;


To balance the relation between h(n) and g(n), two parameters can be adjusted: 'greedy_h', 'greedy_g'.


To use the searching algorithm, set the environment using the code below in 'main.m'

-- Setting the start coordinates:
xStart = 1.0;
yStart = 1.0;

-- Setting the target coordinates:
xTarget = 28.0;
yTarget = 23.0;

-- Setting the map information:
- map size:
MAX_X = 30;
MAX_Y = 25;
- map layout: (note that the map not a fixed map. To change the map layout,
refer to 'obstacle_map.m'.)
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);

-- Applying A* searching to generate a path.
path = A_star_search(map, MAX_X,MAX_Y,greedy_h,greedy_g);

