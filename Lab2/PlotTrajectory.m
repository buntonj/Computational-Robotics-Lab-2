function PlotTrajectory(robotCornerTrajectories,obstacles,goal,distanceThreshold)
PlotObstacles(obstacles)
PlotGoal(goal,distanceThreshold)
axis square equal tight manual
axis([0 5,0 5]);
