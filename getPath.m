function [path] = getPath(start,finish)

% Getting the map

image = imread('NormalFloorPlan.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 1;
normal_map = binaryOccupancyMap(bwimage,80);
map = binaryOccupancyMap(bwimage,80);

% Inflating walls to avoid collisions with robot
radius = 0.5;
inflate(map,0.5)

%Forming paths between various nodes
prmSimple = mobileRobotPRM(map,400);

path = findpath(prmSimple, start, finish);

end


