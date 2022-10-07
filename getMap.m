% Loading map 
function [obstacle_map,normal_map] = getMap()

%Creating normal_map with obstacle 
image = imread('NormalFloorPlan.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 1;
normal_map = binaryOccupancyMap(bwimage,80);
%map = binaryOccupancyMap(bwimage,150);

%inflate(map,0.5);

image = imread('Obstacle1.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 1;
obstacle_map = binaryOccupancyMap(bwimage,80);

%show(normal_map)

end