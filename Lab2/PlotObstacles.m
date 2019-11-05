function PlotObstacles(obstacles)
f = [1:length(obstacles{1,1})];
patch('Faces',f,'Vertices',obstacles{1,1},'EdgeColor','black','FaceColor','none','LineWidth',2)
hold on
for i=2:length(obstacles)
f = [1:length(obstacles{i,1})];
patch('Faces',f,'Vertices',obstacles{i,1},'EdgeColor','black','FaceColor','black','LineWidth',1)
end
