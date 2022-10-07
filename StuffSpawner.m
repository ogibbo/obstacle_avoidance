% Function to generate random obstacles on the map
function [MapwithStuff]=StuffSpawner(OccupancyMap,NumStuff,resolution)
    StuffCoords=zeros(999,2);
    for i=1:NumStuff
        StuffX=rand*(OccupancyMap.XWorldLimits(1,2)-OccupancyMap.XWorldLimits(1,1))+OccupancyMap.XWorldLimits(1,1);
        StuffY=rand*(OccupancyMap.YWorldLimits(1,2)-OccupancyMap.YWorldLimits(1,1))+OccupancyMap.YWorldLimits(1,1);
        StuffCoords(1,:)=[StuffX,StuffY];
        localoccupancy=checkOccupancy(OccupancyMap,[StuffX,StuffY]);
        while localoccupancy==1
            StuffX=rand*(OccupancyMap.XWorldLimits(1,2)-OccupancyMap.XWorldLimits(1,1))+OccupancyMap.XWorldLimits(1,1);
            StuffY=rand*(OccupancyMap.YWorldLimits(1,2)-OccupancyMap.YWorldLimits(1,1))+OccupancyMap.YWorldLimits(1,1);
            localoccupancy=checkOccupancy(OccupancyMap,[StuffX,StuffY]);
        end
        
        for j=[StuffX:1/resolution:StuffX+24/resolution]
            for k=[StuffY:1/resolution:StuffY+24/resolution]
                setOccupancy(OccupancyMap,[j,k],1);
            end
        end
        
%         x = [StuffX:1/resolution:(StuffX+24/resolution)]';
%         y = [StuffY:1/resolution:(StuffY+24/resolution)]';
%         setOccupancy(OccupancyMap,[x y],ones(25,1));
        
        MapWithStuff=OccupancyMap;
        
    end