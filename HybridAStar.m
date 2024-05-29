clear; close all;
load exampleMaps.mat;

% Define the map dimensions and obstacles
map = binaryOccupancyMap(complexMap);

%Create a state validator object for collision checking.
validator = validatorOccupancyMap;

%Assign the map to the state validator object.
validator.Map = map;

%Plan and Visualize Path
%Initialize the plannerHybridAStar object with the state validator object. Specify the MinTurningRadius and MotionPrimitiveLength properties of the planner.
planner = plannerHybridAStar(validator,'MinTurningRadius',3,'MotionPrimitiveLength',4);
%Define start and goal poses for the vehicle as [x, y, theta] vectors. x and y specify the position in meters, and theta specifies the orientation angle in radians.
startPose = [3 3 pi/2]; % [meters, meters, radians]
goalPose = [32 38 0];
%Plan a path from the start pose to the goal pose.
refpath = plan(planner,startPose,goalPose);
%Visualize the path using show function.
figure;
show(planner);
hold on;

% Define the kinematic parameters of the robot
r= differentialDriveKinematics("TrackWidth",0.5,"VehicleInputs","VehicleSpeedHeadingRate");
r.WheelSpeedRange = [-10 10]*2*pi;
%t=0:0.1:1000;
init=startPose;%initial conditions

% Path following controller
controller= controllerPurePursuit('DesiredLinearVelocity', 1, 'MaxAngularVelocity', 2);
controller.Waypoints = refpath.States(:,1:2); 
controller.LookaheadDistance = 0.5;

pose =init;% Initialize the robot's initial pose
figure; % Create a figure for visualization
show(map);
hold on;
plot(refpath.States(:,1), refpath.States(:,2),'r--','LineWidth',2);
plot(startPose(1),startPose(2),'go','MarkerSize', 10, 'LineWidth', 2); % Start point
plot(goalPose(1),goalPose(2),'ro','MarkerSize', 10, 'LineWidth', 2); % Goal point

tic%to calculate elapsed time
i= 1;
while (1)
    [v,omega] = controller(pose(i,:)); % Compute the controller outputs
    vel = derivative(r, pose(i,:),[v,omega]);% Compute the velocity
    pose(i+1,:)=pose(i,:) + vel' * 0.1; % Update the robot pose   
    plot(pose(i,1),pose(i,2),'bp');% Plot the robot's current position

    diff = pose(i,1:2) - goalPose(1:2);%check whether robot's current position is within a certain threshold of distance
    diff1= dot(diff,diff);
    diff2= sqrt(diff1);

    if diff2 < 0.5 %position is within threshold limits
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

plot(pose(end,1), pose(end,2),'bx', 'MarkerSize', 10,'LineWidth',2);% Plot the final position of the robot
toc %end of time
