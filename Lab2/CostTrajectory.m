function cost = CostTrajectory(trajectory,parentNodeCoords,parentNodeCost)
cost = norm(trajectory-parentNodeCoords) + parentNodeCost;
end