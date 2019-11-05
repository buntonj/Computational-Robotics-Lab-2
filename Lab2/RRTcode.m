clc
clear
close all
profile on


%% Initialization of variables
global u; %We use this as u is computed inside the anonymous handle used for
%ode45 and we dont want to loose it

roomLength=5;
roomWidth=5;
robotLength=0.1;
robotWidth=0.09;
robotWheelRadius=0.025;
roomBounds=[0 roomLength; 0 roomWidth];
roomVertices=[0 roomWidth; roomLength roomWidth; roomLength 0; 0 0];
robotDriveTime = 1; % simulation time at each RRT iteration
saveToPdf = false; % if true will save a pdf every 100 steps of the RRT.
%Requires export_fig toolbox, download from https://www.mathworks.com/matlabcentral/fileexchange/23629-export_fig

filename = strcat('RRT','.gif'); % Name of the gif we will create
loadObstacles = false; % Set true if you want to re-use previously generated obstacles
randomPolygons = false;
randomPolygonFaces = 5;
randomPolygonsNumber = 5;
randomRectangles = true;
randomRectanglesNumber = 30;


state=[0.1;0.1;0]; % Initial State
goal=[4;1;0];
distanceThreshold = 0.1; % How close to the goal we need to get to consider it a success
counter = 2;
distanceToGoal = 1e3; % Initialize current cost to be large
% First node in tree
nodes(1).coord = state(1:3)';
nodes(1).parent = 0;
nodes(1).trajectory = [state(1:3)';state(1:3)'];
nodes(1).input = [0,0;0,0];








%% Load or create obstacles
if loadObstacles
    load('obstacles.mat')
else
    obstacles=AddObstacle(roomVertices);
    obstacleVertices = [2.5 1.5; 3.5 1.5; 3.5 0; 2.5 0];
    obstacles=AddObstacle(obstacleVertices,obstacles);
    
    if randomPolygons
        for i=1:randomPolygonsNumber
            createInArea = roomBounds;
            obstacleVertices = CreateRandomPoligon(randomPolygonFaces,createInArea);
            obstacles=AddObstacle(obstacleVertices,obstacles);
        end
    end
    if randomRectangles
        for i=1:randomRectanglesNumber
            createInArea = roomBounds;
            obstacleVertices = CreateRandomRectangle(roomBounds);
            obstacles=AddObstacle(obstacleVertices,obstacles);
        end
    end
    save('obstacles.mat','obstacles')
end






%% Plot arena and goal
figure('units','normalized','outerposition',[0 0 1 1]);
hold on;
PlotObstacles(obstacles)
PlotGoal(goal,distanceThreshold)
radiusStartArea = 0.05;
PlotStart(state,radiusStartArea)
axis square equal tight manual
axis([0 5,0 5]);
% Print to gif
F = getframe;
im = frame2im(F);
[imind,cm] = rgb2ind(im,256);
imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
if saveToPdf
    s = strcat('RRT-Iteration-',num2str(0),'.pdf');
    export_fig(s,'-q101')
end

%% RRT
while distanceToGoal>distanceThreshold
    % Randomly sample space
    sample = [roomBounds(1,1) + (-roomBounds(1,1)+roomBounds(1,2))*rand(1),roomBounds(2,1)+(-roomBounds(2,1)+roomBounds(2,2))*rand(1),rand(1)*2*pi];
    auxVect=cat(1,nodes.coord);
    % Search closest node
    sampleDistance = vecnorm(auxVect(:,1:2)-sample(1:2),2);
    closestNode = find(sampleDistance == min(sampleDistance));
    % Compute trajectory from closest node to sample point
    [newState,trajectory,u] = RobotDynamicsStep(nodes(closestNode(1)).coord,sample,robotDriveTime);
    % Check if we reached goal
    goalDistance = vecnorm(trajectory(:,1:2)-goal(1:2)',2);
    % If we reached goal we stop
    if min(goalDistance)<=distanceThreshold
        closestPoint=find(goalDistance==min(goalDistance));
        trajectory = trajectory(1:closestPoint,:);
        trajectory = interp1(linspace(0,robotDriveTime,size(trajectory,1))',trajectory,linspace(0,robotDriveTime,100*robotDriveTime)');
        u = u(1:closestPoint,:);
        u = interp1(linspace(0,robotDriveTime,size(u,1))',u,linspace(0,robotDriveTime,100*robotDriveTime+1)');
        newState = trajectory(end,:);
    end
    % Check there were no collisions in the trajectory by checking if any
    % of the robots corners hit an obstacle
    robotCornerTrajectories = ComputeRobotCornerTrajectories(trajectory,robotLength,robotWidth,robotWheelRadius);
    collision = CheckCollisions(robotCornerTrajectories, obstacles);
    % If there where no collisions we add the end state of our trajectory
    % to the graph as a new node
    if ~collision
        nodes(counter).coord = newState;
        nodes(counter).parent = closestNode(1);
        nodes(counter).trajectory = trajectory(2:end,:);
        nodes(counter).input = u(2:end,:);
        counter = counter+1;
        % Plot new node and edge, wont show in plot until command "drawnow"
        plot(trajectory(:,1),trajectory(:,2),'b')
        % Compute distance to goal, if we reached it we will exit
        distanceToGoal = norm(trajectory(end,1:2)-goal(1:2)');
        % Refresh plot and add frame to gif every 20 iterations
        if ~mod(counter,20)
            drawnow
            F = getframe;
            im = frame2im(F);
            [imind,cm] = rgb2ind(im,256);
            imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append');
            if ~mod(counter,100)
                % print iteration number every 100 iterations
                disp(strcat('Iterarion:',{' '},num2str(counter)))
                if saveToPdf
                    s = strcat('RRT-Iteration-',num2str(counter),'.pdf');
                    export_fig(s,'-q101')
                end
            end
        end
    end
    
end

%% Find path from goal to start
backtrack = 1;
counter2 = 1;
nodeList(1) = length(nodes);
while backtrack
    counter2 = counter2 + 1;
    nodeList(counter2) = nodes(nodeList(counter2-1)).parent;
    % If at initial node then stop backtracking and eliminate parent 0
    if nodeList(counter2) == 0
        backtrack = false;
        nodeList(end) = [];
    end
end

%% Build trajectory by stitching all trajectories in chain, is valid as all nodes are the endpoints of the trajectory from their parents
trajectoryFinal = cat(1,nodes(flip(nodeList)).trajectory);
% Plot path
plot(trajectoryFinal(:,1),trajectoryFinal(:,2),'g','Linewidth',1.5)
% Create frame for gif
F = getframe;
im = frame2im(F);
[imind,cm] = rgb2ind(im,256);
imwrite(imind,cm,filename,'gif','DelayTime',1,'WriteMode','append');

% Print to PDF
if saveToPdf
    s = strcat('RRT-Iteration-',num2str(counter),'.pdf');
    export_fig(s,'-q101')
end
% Profile lets you analize the computational costs
profile viewer
profile off
