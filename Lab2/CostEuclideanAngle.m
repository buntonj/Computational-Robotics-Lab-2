function cost = CostEuclideanAngle(trajectory,parentNodeCoords,parentNodeCost)
cost = norm(trajectory(end,:)-parentNodeCoords(:)) + parentNodeCost;
end