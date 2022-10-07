function [timetaken,DeliveryOutcome,EndPose] = PathFollowing(path,current_pose)

%%Inputs:
% path - waypoints for robot to follow (eg from one apartment to the next
% apartment)[x1 y1; x2 y2; x3 y3; ...]
%normal_map - the binary occupancy grid to be used for visualisation
%current_pose , pose of the robot when the function is called [3x1]

%Output - DeliveryOutcome (1 if success, 0 if not)
%EndPose - Pose of the robot at the end of the delivery [3x1]
%SimVariable - optional variable for simulation

%% Define Vehicle
wheelRadius = 0.1;             % Wheel radius [m]
frontLen = 0.35;               % Distance from CG to front wheels [m]
rearLen = 0.35;                 % Distance from CG to rear wheels [m]
RobotRadius = 0.45;
vehicle = FourWheelSteering(wheelRadius,[frontLen rearLen]);

% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:180;        % Time array

% Initial conditions
initPose = current_pose;            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,51);
lidar.maxRange = 10;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map_with_obstacles';
attachLidarSensor(viz,lidar);

%% Path planning and following

% Create waypoints
waypoints = [initPose(1:2)'; path];
             
         
%Setting maximum valocity of the robot

vmax = 0.7;

% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.4;
controller.DesiredLinearVelocity = vmax;
controller.MaxAngularVelocity = 1.5;

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.3 3];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [3 7]; 
vfh.RobotRadius = RobotRadius;
vfh.SafetyDistance = 0;
vfh.MinTurningRadius = 1.2;

%Creating while loop so that the robot eventually stops
robot_initial_location = path(1,:);
robot_final_location = path(end,:);

goalRadius = 0.4;
distanceToGoal = norm(robot_initial_location - robot_final_location);

%% Simulation loop
r = rateControl(1/sampleTime);
i = 0;
for idx = 2:numel(tVec) 
    
    % Checking if robot has reached goal
    if distanceToGoal < goalRadius
        
        random_number = rand;

        % Assuming 95% of packages will be retrieved by recipient
        if random_number < 0.95
        DeliveryOutcome = 1;
        else
        DeliveryOutcome = 0;
        end
        
        timetaken = tVec(idx);
        EndPose = curPose;
        
        return
    end
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
    
    %Minimum value of lidar
    minimum = min(ranges,[],'omitnan');
    
    %range threshold for changing velocity 
    range = 1.5;
    zero_velocity_distance = 0.4;
    safety_stop_distance = 0.35;
    
    % Changing velocity of robot depending on distance to nearest obstacle
    if minimum < range
        controller.DesiredLinearVelocity = abs((vmax/(range-zero_velocity_distance))*(minimum-0.5))+0.2;
        if minimum < safety_stop_distance
            disp('Too close to obstacle')
            DeliveryOutcome = 0;
            timetaken = tVec(idx);
            EndPose = curPose;
            return
        end
    else
        controller.DesiredLinearVelocity = vmax;
    end
        
    %Getting remaining distance to Goal
    curPosition = curPose(1:2)';
    distanceToGoal = norm(curPosition - robot_final_location);
    
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef,lookAheadPt] = controller(curPose);
    
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    [wheelSpeeds,steerAngles] = ...
                 inverseKinematicsZeroSideslip(vehicle,vRef,wRef);
                 %inverseKinematicsFrontSteer(vehicle,vRef,wRef);
    
    % Compute the velocities
    velB = forwardKinematics(vehicle,wheelSpeeds,steerAngles);
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Robot will move in opposite direction if more efficient than turning around
    difference = abs(steerDir-targetDir);
    number = difference/(2*pi);
    
    integ=floor(number);
    fract=number-integ;
    
    if fract > 0.5 && i<3
        i = i+1;
        pause(2)
       pose(3,idx) = pose(3,idx) +pi;
    end
    
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    
    % Update control input arrays
    wArray(:,idx-1) = wheelSpeeds;
    phiArray(:,idx-1) = steerAngles;
    waitfor(0.7);
    
    %vfhplot = figure;
    %figure(vfhplot)
    %show(vfh);
    
    EndPose = curPose;
    timetaken = tVec(idx);
    DeliveryOutcome = 0;
   
end

end
