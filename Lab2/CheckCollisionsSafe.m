function collision = CheckCollisions(robotCornerTrajectories, obstacles, walls)


cornerPositions = [ [robotCornerTrajectories{1};robotCornerTrajectories{2};robotCornerTrajectories{3};robotCornerTrajectories{4}] ones(4*length(robotCornerTrajectories{1}),1)]';
statesToCheck=length(cornerPositions);
facesToCheck = length(cat(1,obstacles{:,2}));
obstacleFacesNormals = repmat(cat(1,obstacles{:,2}),statesToCheck,1);
[A,B] = find(kron(speye(statesToCheck),ones(facesToCheck,3)));
AMatrix = sparse(A,B,reshape(permute(reshape(obstacleFacesNormals, facesToCheck, statesToCheck, 3), [2 1 3]), statesToCheck, facesToCheck*3)');
[r,c] = size(cornerPositions);
i     = 1:numel(cornerPositions);
j     = repmat(1:c,r,1);
cornerPositions = sparse(i',j(:),cornerPositions(:));
% % Multiplicacion en sparse
InOut = nonzeros(AMatrix*cornerPositions);
InOutOrdered=InOut(1:facesToCheck:end);
for i=2:facesToCheck
   InOutOrdered=[InOutOrdered InOut(i:facesToCheck:end)];
end
stateCollision = InOutOrdered<0;
collision = max(sum(InOut,2) == 6); % Si sumo seis estoy adentro de todas las caras