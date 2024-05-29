clear ; close all;

% Define the map dimensions and obstacles
load exampleMaps.mat;
map = binaryOccupancyMap(complexMap);

% Define start and goal positions
start = [3,3];
goal= [32,38];
% Visualize the map
figure;
show(map);
hold on;
plot(start(1),start(2));
plot(goal(1),goal(2));

% Create the PRM object
prm=mobileRobotPRM(map,200);%200 nodes

% Find a path using PRM
path= findpath(prm,start,goal);

% Check if a path is found
if isempty(path)
    disp('No Path Found');
else
    disp('Path Found');
    % Plot the path
    plot(path(:,1),path(:,2),'b-','LineWidth',2);
end

%Path smoothing
options = optimizePathOptions;
options.ObstacleSafetyMargin = 1;
smooth = optimizePath(path, map, options);
plot(smooth(:,1), smooth(:,2),'r--','LineWidth',2);
legend("","","PRM", "Smoothed Path");

% Define the kinematic parameters of the robot
r= differentialDriveKinematics("TrackWidth",0.5,"VehicleInputs","VehicleSpeedHeadingRate");
r.WheelSpeedRange = [-10 10]*2*pi;
%t=0:0.1:1000;
init=[start,0];%initial conditions

% Path following controller
controller= controllerPurePursuit('DesiredLinearVelocity', 1, 'MaxAngularVelocity', 2);
controller.Waypoints = [smooth(:,1) smooth(:,2)]; % path;
controller.LookaheadDistance = 0.5;% 0.5;
pose =init;% Initialize the robot's initial pose
figure; % Create a figure for visualization
show(map);
hold on;
%plot(path(:,1), path(:,2),'r--','LineWidth',2);
plot(start(1),start(2),'go','MarkerSize', 10, 'LineWidth', 2); % Start point
plot(goal(1),goal(2),'ro','MarkerSize', 10, 'LineWidth', 2); % Goal point
plot(smooth(:,1), smooth(:,2),'r--','LineWidth',2);
tic %to calculate elapsed time

% Simulate the robot motion
i= 1;
while (1)
    [v,omega] = controller(pose(i,:)); % Compute the controller outputs
    vel = derivative(r, pose(i,:),[v,omega]);% Compute the velocity
    pose(i+1,:)=pose(i,:) + vel' * 0.1; % Update the robot pose   
    plot(pose(i,1),pose(i,2),'bp');% Plot the robot's current position

    diff = pose(i,1:2) - goal;%check whether robot's current position is within a certain threshold of distance
    diff1= dot(diff,diff);
    diff2= sqrt(diff1);
    if diff2 < 0.1 %position is within threshold limits
        destReach = 1;
    else
        destReach = 0;
    end    
    if destReach == 1
        break;
    end        
    drawnow;
    i= i+1;
end
plot(pose(end,1), pose(end,2),'bx', 'MarkerSize', 10,'LineWidth',1.3);% Plot the final position of the robot
toc %end of time
