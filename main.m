%% Run Once
clc; clear all

resolution = 80;

% Defining waypoints
start = [2.6,1.6];
finish = [9.4,6.6];

% Finding path using PRM
path = getPath(start,finish);

% clear existing normal_map
clear normal_map
[one_obstacle_map,normal_map] = getMap();

% clear existing map_with_obstacles
clear map_with_obstacles
% drawing random obstacles on the map
map_with_obstacles = StuffSpawner(normal_map,10,resolution);

% Initial conditions for the robot
initPose = [path(1,1);path(1,2);pi/2];       % Initial pose (x y theta)

[timetaken,DeliveryOutcome,OutputPose] = PathFollowing(path,initPose);


 





