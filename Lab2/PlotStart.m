function PlotStart(state,distanceThreshold)
rectangle('Position',[state(1)-distanceThreshold,state(2)-distanceThreshold,distanceThreshold*2,distanceThreshold*2],'Curvature',[1,1], 'FaceColor','g')
