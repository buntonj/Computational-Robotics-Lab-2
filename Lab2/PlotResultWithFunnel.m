%% Compute final trajectory according to dynamics
% Run after rrt*
% PathSubfolders

% Determine where your m-file's folder is.
folder = fileparts(which('get_rszcarimg.m')); 
% Add that folder plus all subfolders to the path.
addpath(genpath(folder));

finalTrajectory = [];
% Give enough time for controller to reach nodes
time=2;
for i=1:length(nodeList)-1
    [newState,trajectory,u] = RobotDynamicsStep(nodes(nodeList(i)).coord,nodes(nodeList(i+1)).coord,time);
    finalTrajectory=[finalTrajectory;trajectory];
end
% Plot arena, goal, starting area, obstacles and center trajectory
figure
PlotObstacles(obstacles)
PlotGoal(goal,distanceThreshold)
PlotStart(state,radiusStartArea)
ReplotTree
axis square equal tight manual
axis([0 5,0 5]);
hold on
plot(finalTrajectory(:,1),finalTrajectory(:,2),'g','Linewidth',1.5)

% Plot funnel
rszwh  = [30 30];
carwh  = 2*[0.05 0.045];
[carrsz, trrsz] = get_rszcarimg('car.png', rszwh);

for i=1:size(finalTrajectory,1)
    carpos=finalTrajectory(i,:);
    [xmesh_grid, ymesh_grid] = get_carmeshgred(carpos, carwh, rszwh);
    hold on
    c = surf('xdata', xmesh_grid, 'ydata', ymesh_grid, 'zdata', zeros(rszwh(1), rszwh(2)), 'cdata', carrsz, 'AlphaData', trrsz , 'FaceAlpha', 'texture', 'FaceColor', 'texturemap' , 'EdgeColor','None', 'LineStyle', 'None');
    alpha(0.3)
end