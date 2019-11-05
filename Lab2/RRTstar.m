clc
clear
close all
profile on


%% Initialization of variables
global u; %We use this as u is computed inside the anonymous handle used for
%ode45 and we dont want to loose it
runMinimumLength = 3000; % We will run RRT* for at least some number of steps to get an efficient path
rrtStarSearchRadius = 0.05; % We will search in a circle of this radius when rewiring the graph
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

filename = strcat('RRTStar','.gif'); % Name of the gif we will create
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
goalReached = false ; % We assume we dont start at the goal
endNode = [];
% First node in tree
nodes(1).coord = state(1:3)';
nodes(1).parent = 0;
nodes(1).trajectory = [state(1:3)';state(1:3)'];
nodes(1).input = [0,0;0,0];
nodes(1).cost = [0];


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
    if saveToPdf
        s = strcat('RRTStar-Iteration-',num2str(0),'.pdf');
        export_fig(s,'-q101')
    end
end

%% RRT*
while ~goalReached || counter <= runMinimumLength
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
    % If we reached goal we stop driving, but we continue sampling other
    % points
    if min(goalDistance)<=distanceThreshold
        closestPoint=find(goalDistance==min(goalDistance));
        % If the closest node to the sample point is already at the goal
        % we sample a new point.
        if min(closestPoint)==1
            continue
        end
        % We interpolate the trajectory so that all edges have 100 points
        % in their trajectories, this simplifies things later.
        trajectory = trajectory(1:closestPoint,:);
        trajectory = interp1(linspace(0,robotDriveTime,size(trajectory,1))',trajectory,linspace(0,robotDriveTime,100*robotDriveTime+1)');
        trajectory = trajectory(2:end,:);
        u = u(1:closestPoint,:);
        u = interp1(linspace(0,robotDriveTime,size(u,1))',u,linspace(0,robotDriveTime,100*robotDriveTime+1)');
        % The new node is the one that reached the goal
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
        nodes(counter).trajectory = trajectory(1:end,:);
        nodes(counter).input = u(2:end,:);
        nodes(counter).cost = CostEuclidean(trajectory,nodes(closestNode(1)).coord,nodes(nodes(counter).parent).cost);
        % Compute distance to goal, if we reached it we record that there
        % is already at least one path to the goal.
        distanceToGoal = norm(trajectory(end,1:2)-goal(1:2)');
        if distanceToGoal<distanceThreshold
            goalReached = true ;
            % We record the tag of the node inside the goal area, we keep
            % record of new ones that might appear
            endNode = [endNode ; counter];
        end
        %% Rewire tree
        % Search for neighbours in range
        nodeDistance = vecnorm(auxVect(:,1:2)-newState(1:2),2);
        nodesInRange = find(nodeDistance <= rrtStarSearchRadius);
        % Check costs from rewiring each neighbour in range
        if nodesInRange
            clear newParents
            for i=1:length(nodesInRange)
                trajectory=[nodes(nodesInRange(i)).coord;nodes(counter).coord];
                trajectory = interp1([0,robotDriveTime],trajectory,linspace(0,robotDriveTime,100*robotDriveTime+1)');
                % Store data in case we do rewire
                newParents(i).cost = CostEuclidean(trajectory,nodes(nodesInRange(i)).coord,nodes(nodesInRange(i)).cost);
                newParents(i).trajectory = trajectory(2:end,:);
                newParents(i).input = u(2:end);
                newParents(i).parent = nodesInRange(i);
                newParents(i).coords = newState;
            end
            % If at least one of the neighbours provides a lower cost we rewire
            if min(cat(1,newParents.cost))< nodes(counter).cost
                % We order the neighbours by cost
                [costValues,index]=sort(cat(1,newParents.cost));
                
                % check for obstacles, we start with the node that has the
                % lowest cost, if there is an obstacle we try with the next
                % one and so on
                noObstacles = 0;
                obstacleCheckPassCounter = 0;
                while ~noObstacles
                    if obstacleCheckPassCounter == length(index)
                        obstacleCheckPassCounter = obstacleCheckPassCounter+1;
                        break
                    end
                    obstacleCheckPassCounter = obstacleCheckPassCounter+1;
                    robotCornerTrajectories = ComputeRobotCornerTrajectories(newParents(index(obstacleCheckPassCounter)).trajectory,robotLength,robotWidth,robotWheelRadius);
                    newcollision = CheckCollisions(robotCornerTrajectories, obstacles);
                    if ~newcollision || costValues(obstacleCheckPassCounter)>nodes(counter).cost
                        noObstacles = 1;
                    end
                end
                % If the first neightbour such that there are no obstacles
                % between it and the new node gives a lower cost for the
                % new node, we rewire the new node to it.
                if obstacleCheckPassCounter <= length(index)
                    bestParent = index(obstacleCheckPassCounter);
                    nodes(counter).coord = newParents(bestParent).coords;
                    nodes(counter).parent = newParents(bestParent).parent;
                    nodes(counter).trajectory =  newParents(bestParent).trajectory(1:end,:);
                    nodes(counter).input = newParents(bestParent).input(2:end,:);
                    nodes(counter).cost = newParents(bestParent).cost;
                end
            end
        end
        % We check if any of the neighbours sees its cost go down by
        % rewiring to the new node
        for i  = nodesInRange'
            trajectory = [nodes(counter).coord;nodes(i).coord];
            trajectory = interp1([0,robotDriveTime],trajectory,linspace(0,robotDriveTime,100*robotDriveTime+1)');
            candidateCost = CostEuclidean(trajectory,nodes(counter).coord,nodes(counter).cost);
            robotCornerTrajectories = ComputeRobotCornerTrajectories(trajectory,robotLength,robotWidth,robotWheelRadius);
            newcollision = CheckCollisions(robotCornerTrajectories, obstacles);
            % If the cost would decrease and we dont have any collisions, we
            % rewire
            if (candidateCost < nodes(i).cost) && ~newcollision
                nodes(i).cost = candidateCost;
                nodes(i).parent = counter;
                nodes(i).trajectory = trajectory(2:end,:);
            end
        end
            
        % Go to next RRT* iteration
        counter = counter+1;
        
        % Refresh tree plot, recompute best path if one was found and
        % add frame to gif every 100 iterations
        if ~mod(counter,100)
            disp(num2str(counter))
            ReplotTree
            drawnow
            
            % Recompute best path
            if goalReached
                backtrack = 1;
                counter3 = 1;
                bestEndNode = endNode(find(cat(2,nodes(endNode).cost)==min(cat(2,nodes(endNode).cost))));
                % If at initial node then stop backtracking and eliminate parent 0
                nodeList = [];
                nodeList(1) = bestEndNode;
                while backtrack
                    counter3 = counter3 + 1;
                    nodeList(counter3) = nodes(nodeList(counter3-1)).parent;
                    if nodeList(counter3) == 0
                        backtrack = false;
                        nodeList(end) = [];
                    end
                end
                %% Build trajectory by stitching all trajectories in chain, is valid as all nodes are the endpoints of the trajectory from their parents
                trajectoryFinal = cat(1,nodes(flip(nodeList)).trajectory);
                % Replot path
                if exist('routeToGoal','var')
                    delete(routeToGoal);
                end
                routeToGoal = plot(trajectoryFinal(:,1),trajectoryFinal(:,2),'g','Linewidth',1.5);
            end
            % Printo to gif and pdf
            F = getframe;
            im = frame2im(F);
            [imind,cm] = rgb2ind(im,256);
            imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append');
            if ~mod(counter,500)
                if saveToPdf
                    s = strcat('RRTStar-Iteration-',num2str(counter),'.pdf');
                    export_fig(s,'-q101')
                end
            end
            
        end
    end
    
end


backtrack = 1;
counter3 = 1;
bestEndNode = endNode(find(cat(2,nodes(endNode).cost)==min(cat(2,nodes(endNode).cost))));
nodeList = [];
nodeList(1) = bestEndNode;
while backtrack
    counter3 = counter3 + 1;
    nodeList(counter3) = nodes(nodeList(counter3-1)).parent;
    if nodeList(counter3) == 0
        backtrack = false;
        % If at initial node then stop backtracking and eliminate parent 0
        nodeList(end) = [];
    end
end

%% Build trajectory by stitching all trajectories in chain, is valid as all nodes are the endpoints of the trajectory from their parents
trajectoryFinal = cat(1,nodes(flip(nodeList)).trajectory);
% Replot path
if exist('routeToGoal','var')
    delete(routeToGoal);
end
routeToGoal = plot(trajectoryFinal(:,1),trajectoryFinal(:,2),'g','Linewidth',1.5);

% Printo to gif and pdf
F = getframe;
im = frame2im(F);
[imind,cm] = rgb2ind(im,256);
imwrite(imind,cm,filename,'gif','DelayTime',1,'WriteMode','append');

if saveToPdf
    s = strcat('RRTStar-Iteration-',num2str(counter),'.pdf');
    export_fig(s,'-q101')
end
PlotResultWithFunnel
profile viewer
profile off