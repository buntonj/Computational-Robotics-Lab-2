function cost = CostEuclidean(trajectory,parentNodeCoords,parentNodeCost)
cost = norm(trajectory(end,1:2)-parentNodeCoords(1:2)) + parentNodeCost;
end