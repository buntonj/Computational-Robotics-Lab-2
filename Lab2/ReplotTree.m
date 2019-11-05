treeEdges = reshape(cat(1,nodes(2:end).trajectory),100,[]);
if exist('h','var')
delete(h);
end
h = plot(treeEdges(:,1:end/3),treeEdges(:,(end/3+1):2*end/3),'b');

